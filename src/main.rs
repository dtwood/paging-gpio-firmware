#![no_std]
#![no_main]
#![allow(deprecated)]
#![allow(non_camel_case_types)]

use core::fmt::Write;
use core::{convert::TryInto, panic::PanicInfo};
use cortex_m_rt::exception;
use heapless::{consts, String};
use rtic::cyccnt::Duration;
use smoltcp::iface::{EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache};
use smoltcp::socket::{SocketHandle, SocketSet, SocketSetItem, TcpSocket, TcpSocketBuffer};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr};
use tm4c129x_hal::ethernet::EthernetDevice;
use tm4c129x_hal::gpio;
use tm4c129x_hal::prelude::*;
use tm4c129x_hal::sysctl::SysctlExt;
use tm4c129x_hal::sysctl::{CrystalFrequency, Oscillator, PllOutputFrequency, SystemClock};
use tm4c129x_hal::tm4c129x;

#[allow(unused_macros)]
macro_rules! dbg {
    () => {
        log::debug!("[{}:{}]", file!(), line!());
    };
    ($val:expr) => {
        // Use of `match` here is intentional because it affects the lifetimes
        // of temporaries - https://stackoverflow.com/a/48732525/1063961
        match $val {
            tmp => {
                log::debug!(
                    "[{}:{}] {} = {:?}",
                    file!(),
                    line!(),
                    stringify!($val),
                    &tmp
                );
                tmp
            }
        }
    };
}

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("IRQ: {}", irqn);
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    log::error!("Application panicked: {}", info);
    cortex_m::asm::bkpt();

    loop {
        cortex_m::asm::wfi();
    }
}

const NUM_BUTTONS: usize = 2;

/// Get port between 49152 and 65535
fn get_ephemeral_port() -> u16 {
    let rand = unsafe { (*tm4c129x::HIB::ptr()).rtcss.read().bits() };

    let mut rand = rand + 49142;
    while rand > 65535 {
        rand -= 65535 - 49142;
    }

    rand as u16
}

#[rtic::app(device = tm4c129x_hal::tm4c129x, peripherals = true, monotonic=rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        IFACE: EthernetInterface<'static, 'static, 'static, EthernetDevice>,

        SOCKET_SET: SocketSet<'static, 'static, 'static>,
        CLIENT_HANDLE: SocketHandle,

        HIB: tm4c129x_hal::hib::Hib,
        BUTTONS: [gpio::Pxx<gpio::Input<gpio::PullUp>>; NUM_BUTTONS],
    }

    #[init(spawn = [gpio_poll, keepalive])]
    fn init(mut c: init::Context) -> init::LateResources {
        static mut TCP_CLIENT_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_CLIENT_TX_DATA: [u8; 1024] = [0; 1024];

        static mut IP_ADDRS: Option<[IpCidr; 1]> = None;
        static mut NEIGHBOR_CACHE_ENTRIES: [Option<(IpAddress, Neighbor)>; 8] = [None; 8];
        static mut SOCKET_SET_ENTRIES: [Option<SocketSetItem<'static, 'static>>; 2] = [None, None];

        let sc = c.device.SYSCTL.constrain();
        // let porta = c.device.GPIO_PORTA_AHB.split(&sc.power_control);
        // let portb = c.device.GPIO_PORTB_AHB.split(&sc.power_control);
        // let portc = c.device.GPIO_PORTC_AHB.split(&sc.power_control);
        // let portd = c.device.GPIO_PORTD_AHB.split(&sc.power_control);
        // let porte = c.device.GPIO_PORTE_AHB.split(&sc.power_control);
        let portf = c.device.GPIO_PORTF_AHB.split(&sc.power_control);
        // let portg = c.device.GPIO_PORTG_AHB.split(&sc.power_control);
        // let porth = c.device.GPIO_PORTH_AHB.split(&sc.power_control);
        let portj = c.device.GPIO_PORTJ_AHB.split(&sc.power_control);
        // let portk = c.device.GPIO_PORTK.split(&sc.power_control);
        // let portl = c.device.GPIO_PORTL.split(&sc.power_control);
        // let portm = c.device.GPIO_PORTM.split(&sc.power_control);
        let portn = c.device.GPIO_PORTN.split(&sc.power_control);
        // let portp = c.device.GPIO_PORTP.split(&sc.power_control);
        // let portq = c.device.GPIO_PORTQ.split(&sc.power_control);

        let mut clocks = sc.clock_setup;
        clocks.oscillator = Oscillator::Main(
            CrystalFrequency::_25mhz,
            SystemClock::UsePll(PllOutputFrequency::_120mhz),
        );
        let clocks = clocks.freeze();

        log::info!(
            "Chip information: {:#?}",
            tm4c129x_hal::sysctl::chip_id::get()
        );

        let hib = tm4c129x_hal::hib::Hib::hib(
            c.device.HIB,
            tm4c129x_hal::hib::Source::ExternalCrystal,
            &sc.power_control,
        );

        log::info!("{}: start", hib.get_millis());

        let mut leds: [&mut dyn embedded_hal::digital::OutputPin; 4] = [
            &mut portn.pn1.into_push_pull_output(),
            &mut portn.pn0.into_push_pull_output(),
            &mut portf.pf4.into_push_pull_output(),
            &mut portf.pf0.into_push_pull_output(),
        ];

        for led in &mut leds {
            led.set_high();
            for _ in 0..500_000 {
                cortex_m::asm::nop();
            }
            led.set_low();
        }

        let ephy = tm4c129x_hal::ephy::Peripherals::take().unwrap().EPHY.0;

        let device = EthernetDevice::new(
            &sc.power_control,
            clocks,
            &mut c.core.NVIC,
            c.device.EMAC0,
            ephy,
        );

        let neighbor_cache = NeighborCache::new(&mut NEIGHBOR_CACHE_ENTRIES[..]);

        *IP_ADDRS = Some([IpCidr::new(IpAddress::v4(192, 168, 0, 51), 24)]);

        let iface = EthernetInterfaceBuilder::new(device)
            .ethernet_addr(EthernetAddress([0x00u8, 0x1A, 0xB6, 0x00, 0x02, 0x74]))
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut IP_ADDRS.as_mut().unwrap()[..])
            .finalize();

        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the
        // data statically to verify that it fits into RAM rather than get
        // undefined behavior when stack overflows.
        let client_socket = TcpSocket::new(
            TcpSocketBuffer::new(&mut TCP_CLIENT_RX_DATA[..]),
            TcpSocketBuffer::new(&mut TCP_CLIENT_TX_DATA[..]),
        );

        let mut socket_set = SocketSet::new(&mut SOCKET_SET_ENTRIES[..]);
        let client_handle = socket_set.add(client_socket);

        c.spawn.gpio_poll().unwrap();
        c.spawn.keepalive().unwrap();

        log::info!("{}: boot", hib.get_millis());

        init::LateResources {
            IFACE: iface,
            SOCKET_SET: socket_set,
            CLIENT_HANDLE: client_handle,
            HIB: hib,
            BUTTONS: [
                // porta.pa2.into_pull_up_input().downgrade().downgrade(),
                // porta.pa3.into_pull_up_input().downgrade().downgrade(),
                // porta.pa4.into_pull_up_input().downgrade().downgrade(),
                // porta.pa5.into_pull_up_input().downgrade().downgrade(),
                // porta.pa6.into_pull_up_input().downgrade().downgrade(),
                // porta.pa7.into_pull_up_input().downgrade().downgrade(),
                // portb.pb0.into_pull_up_input().downgrade().downgrade(),
                // portb.pb1.into_pull_up_input().downgrade().downgrade(),
                // portb.pb2.into_pull_up_input().downgrade().downgrade(),
                // portb.pb3.into_pull_up_input().downgrade().downgrade(),
                // portb.pb4.into_pull_up_input().downgrade().downgrade(),
                // portb.pb5.into_pull_up_input().downgrade().downgrade(),
                // portc.pc4.into_pull_up_input().downgrade().downgrade(),
                // portc.pc5.into_pull_up_input().downgrade().downgrade(),
                // portc.pc6.into_pull_up_input().downgrade().downgrade(),
                // portc.pc7.into_pull_up_input().downgrade().downgrade(),
                // portd.pd0.into_pull_up_input().downgrade().downgrade(),
                // portd.pd1.into_pull_up_input().downgrade().downgrade(),
                // portd.pd2.into_pull_up_input().downgrade().downgrade(),
                // portd.pd3.into_pull_up_input().downgrade().downgrade(),
                // portd.pd4.into_pull_up_input().downgrade().downgrade(),
                // portd.pd5.into_pull_up_input().downgrade().downgrade(),
                // portd.pd6.into_pull_up_input().downgrade().downgrade(),
                // porte.pe0.into_pull_up_input().downgrade().downgrade(),
                // porte.pe1.into_pull_up_input().downgrade().downgrade(),
                // porte.pe2.into_pull_up_input().downgrade().downgrade(),
                // porte.pe3.into_pull_up_input().downgrade().downgrade(),
                // porte.pe4.into_pull_up_input().downgrade().downgrade(),
                // porte.pe5.into_pull_up_input().downgrade().downgrade(),
                // portf.pf1.into_pull_up_input().downgrade().downgrade(),
                // portf.pf2.into_pull_up_input().downgrade().downgrade(),
                // portf.pf3.into_pull_up_input().downgrade().downgrade(),
                // portg.pg0.into_pull_up_input().downgrade().downgrade(),
                // portg.pg1.into_pull_up_input().downgrade().downgrade(),
                // porth.ph0.into_pull_up_input().downgrade().downgrade(),
                // porth.ph1.into_pull_up_input().downgrade().downgrade(),
                // porth.ph2.into_pull_up_input().downgrade().downgrade(),
                // porth.ph3.into_pull_up_input().downgrade().downgrade(),
                // portj.pj0.into_pull_up_input().downgrade().downgrade(),
                // portj.pj1.into_pull_up_input().downgrade().downgrade(),
                // portk.pk0.into_pull_up_input().downgrade().downgrade(),
                // portk.pk1.into_pull_up_input().downgrade().downgrade(),
                // portk.pk2.into_pull_up_input().downgrade().downgrade(),
                // portk.pk3.into_pull_up_input().downgrade().downgrade(),
                // portk.pk4.into_pull_up_input().downgrade().downgrade(),
                // portk.pk5.into_pull_up_input().downgrade().downgrade(),
                // portk.pk6.into_pull_up_input().downgrade().downgrade(),
                // portk.pk7.into_pull_up_input().downgrade().downgrade(),
                // portl.pl0.into_pull_up_input().downgrade().downgrade(),
                // portl.pl1.into_pull_up_input().downgrade().downgrade(),
                // portl.pl2.into_pull_up_input().downgrade().downgrade(),
                // portl.pl3.into_pull_up_input().downgrade().downgrade(),
                // portl.pl4.into_pull_up_input().downgrade().downgrade(),
                // portl.pl5.into_pull_up_input().downgrade().downgrade(),
                // portl.pl6.into_pull_up_input().downgrade().downgrade(),
                // portl.pl7.into_pull_up_input().downgrade().downgrade(),
                // portm.pm0.into_pull_up_input().downgrade().downgrade(),
                // portm.pm1.into_pull_up_input().downgrade().downgrade(),
                // portm.pm2.into_pull_up_input().downgrade().downgrade(),
                // portm.pm3.into_pull_up_input().downgrade().downgrade(),
                // portm.pm4.into_pull_up_input().downgrade().downgrade(),
                // portm.pm5.into_pull_up_input().downgrade().downgrade(),
                // portm.pm6.into_pull_up_input().downgrade().downgrade(),
                // portm.pm7.into_pull_up_input().downgrade().downgrade(),
                // portn.pn2.into_pull_up_input().downgrade().downgrade(),
                // portn.pn3.into_pull_up_input().downgrade().downgrade(),
                // portn.pn4.into_pull_up_input().downgrade().downgrade(),
                // portn.pn5.into_pull_up_input().downgrade().downgrade(),
                // portn.pn6.into_pull_up_input().downgrade().downgrade(),
                // portn.pn7.into_pull_up_input().downgrade().downgrade(),
                // portp.pp0.into_pull_up_input().downgrade().downgrade(),
                // portp.pp1.into_pull_up_input().downgrade().downgrade(),
                // portp.pp2.into_pull_up_input().downgrade().downgrade(),
                // portp.pp3.into_pull_up_input().downgrade().downgrade(),
                // portp.pp4.into_pull_up_input().downgrade().downgrade(),
                // portp.pp5.into_pull_up_input().downgrade().downgrade(),
                // portq.pq0.into_pull_up_input().downgrade().downgrade(),
                // portq.pq1.into_pull_up_input().downgrade().downgrade(),
                // portq.pq2.into_pull_up_input().downgrade().downgrade(),
                // portq.pq3.into_pull_up_input().downgrade().downgrade(),
                // portq.pq4.into_pull_up_input().downgrade().downgrade(),
                portj.pj0.into_pull_up_input().downgrade().downgrade(),
                portj.pj1.into_pull_up_input().downgrade().downgrade(),
            ],
        }
    }

    #[idle(resources = [IFACE, SOCKET_SET, HIB])]
    fn idle(c: idle::Context) -> ! {
        let mut hib = c.resources.HIB;
        let mut socket_set = c.resources.SOCKET_SET;
        let iface = c.resources.IFACE;

        loop {
            let _ = socket_set.lock(|socket_set| {
                let now: i64 = hib.lock(|hib| hib.get_millis().try_into().unwrap());
                let _ = iface.poll(socket_set, Instant::from_millis(now));
            });

            // if socket.can_recv() {
            //     let recv = socket.recv(|buffer| (buffer.len(), buffer)).unwrap();
            //     log::info!(
            //         "{}: rx {:?}",
            //         c.resources.HIB.get_millis(),
            //         ::core::str::from_utf8(recv)
            //     );
            // }

            // if let Some(delay) = c.resources.IFACE.poll_delay(
            //     &*c.resources.SOCKET_SET,
            //     rtic::Instant::now().to_smol_instant(),
            // ) {
            //     let wake_at = rtfm::Instant::now().to_smol_instant() + delay;
            //     let _ = rtfm::Instant::now().handle_ethernet(wake_at.to_rtfm_instant());
            // } else {
            //     dbg!();
            // }
        }
    }

    #[task(
        schedule=[keepalive],
        resources=[SOCKET_SET, CLIENT_HANDLE],
        priority=1,
    )]
    fn keepalive(c: keepalive::Context) {
        let mut socket_set = c.resources.SOCKET_SET;
        let mut client_handle = c.resources.CLIENT_HANDLE;

        socket_set.lock(|socket_set| {
            client_handle.lock(|client_handle| {
                let mut socket = socket_set.get::<TcpSocket>(*client_handle);

                if !socket.is_open() {
                    socket
                        .connect(
                            (IpAddress::v4(192, 168, 0, 1), 49280),
                            (IpAddress::Unspecified, get_ephemeral_port()),
                        )
                        .unwrap();
                }

                let s = "devstatus runmode\n";
                if let Ok(true) = socket.send(|buffer| {
                    let bytes = s.as_bytes();

                    if buffer.len() > bytes.len() {
                        buffer[0..bytes.len()].copy_from_slice(bytes);
                        (bytes.len(), true)
                    } else {
                        (0, false)
                    }
                }) {
                    // *old_state = Some(new_state);
                    // log::info!("{}: tx {:?}", c.resources.HIB.get_millis(), s)
                }
            })
        });

        c.schedule
            .keepalive(c.scheduled + Duration::from_cycles(120_000_000))
            .unwrap();
    }

    #[task(
        schedule = [gpio_poll],
        resources=[SOCKET_SET, CLIENT_HANDLE, BUTTONS, HIB],
        priority=2,
    )]
    fn gpio_poll(c: gpio_poll::Context) {
        static mut OLD_STATE: [Option<bool>; NUM_BUTTONS] = [None; NUM_BUTTONS];

        let mut socket = c
            .resources
            .SOCKET_SET
            .get::<TcpSocket>(*c.resources.CLIENT_HANDLE);

        if !socket.is_open() {
            socket
                .connect(
                    (IpAddress::v4(192, 168, 0, 1), 49280),
                    (IpAddress::Unspecified, get_ephemeral_port()),
                )
                .unwrap();
        }

        for (index, (button, old_state)) in c
            .resources
            .BUTTONS
            .iter()
            .zip(OLD_STATE.iter_mut())
            .enumerate()
        {
            let new_state = button.is_low();

            if Some(new_state) != *old_state {
                let mut s = String::<consts::U64>::new();
                if let Ok(true) = socket.send(|buffer| {
                    write!(
                        s,
                        "devstatus runmode\nset MTX:Index_{} 0 0 {}\nset MTX:Index_{} 0 0 {}\n",
                        index % 5 + 1,
                        if new_state { index / 5 + 1 } else { 0 },
                        index % 5 + 6,
                        if new_state { 1 } else { 0 },
                    )
                    .unwrap();

                    let bytes = s.as_bytes();

                    if buffer.len() > bytes.len() {
                        buffer[0..bytes.len()].copy_from_slice(bytes);
                        (bytes.len(), true)
                    } else {
                        (0, false)
                    }
                }) {
                    *old_state = Some(new_state);
                    log::info!("{}: tx {:?}", c.resources.HIB.get_millis(), s)
                }
            }
        }

        c.schedule
            .gpio_poll(c.scheduled + Duration::from_cycles(10_000_000))
            .unwrap();
    }

    extern "C" {
        fn UART6();
        fn UART7();
    }
};
