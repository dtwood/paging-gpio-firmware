#![no_std]
#![no_main]
#![allow(deprecated)]
#![allow(non_camel_case_types)]

use core::fmt::Write;
use core::panic::PanicInfo;
use cortex_m_rt::exception;
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

pub fn get_stdout() -> impl Write {
    unsafe { SERIAL.as_mut().unwrap() }
}

#[macro_export]
macro_rules! println {
    ($fmt:expr) => {
        {
            use core::fmt::Write as _println_Write;
            let _ = writeln!($crate::get_stdout(), $fmt);
        }
    };
    ($fmt:expr, $($arg:tt)*) => {
        {
            use core::fmt::Write as _println_Write;
            let _ = writeln!($crate::get_stdout(), $fmt, $($arg)*);
        }
    }
}

#[allow(unused_macros)]
macro_rules! dbg {
    () => {
        println!("[{}:{}]", file!(), line!());
    };
    ($val:expr) => {
        // Use of `match` here is intentional because it affects the lifetimes
        // of temporaries - https://stackoverflow.com/a/48732525/1063961
        match $val {
            tmp => {
                println!(
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
    // prints the exception frame as a panic message
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("IRQ: {}", irqn);
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    println!("{}", info);

    loop {
        cortex_m::asm::wfi();
    }
}

static mut SERIAL: Option<
    tm4c129x_hal::serial::Serial<
        tm4c129x::UART0,
        tm4c129x_hal::gpio::gpioa::PA1<
            tm4c129x_hal::gpio::AlternateFunction<
                tm4c129x_hal::gpio::AF1,
                tm4c129x_hal::gpio::OpenDrain<tm4c129x_hal::gpio::PullUp>,
            >,
        >,
        tm4c129x_hal::gpio::gpioa::PA0<
            tm4c129x_hal::gpio::AlternateFunction<
                tm4c129x_hal::gpio::AF1,
                tm4c129x_hal::gpio::OpenDrain<tm4c129x_hal::gpio::PullUp>,
            >,
        >,
        (),
        (),
    >,
> = None;

#[rtfm::app(device = tm4c129x_hal::tm4c129x)]
const APP: () = {
    static mut IFACE: EthernetInterface<'static, 'static, 'static, EthernetDevice> = ();

    static mut SOCKET_SET: SocketSet<'static, 'static, 'static> = ();
    // static mut SERVER_HANDLE: SocketHandle = ();
    static mut CLIENT_HANDLE: SocketHandle = ();

    static mut HIB: tm4c129x_hal::hib::Hib = ();

    #[init(spawn = [packet_timer])]
    fn init(c: init::Context) -> init::LateResources {
        // static mut TCP_SERVER_RX_DATA: [u8; 1024] = [0; 1024];
        // static mut TCP_SERVER_TX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_CLIENT_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_CLIENT_TX_DATA: [u8; 1024] = [0; 1024];

        static mut IP_ADDRS: Option<[IpCidr; 2]> = None;
        static mut NEIGHBOR_CACHE_ENTRIES: [Option<(IpAddress, Neighbor)>; 8] = [None; 8];
        static mut SOCKET_SET_ENTRIES: [Option<SocketSetItem<'static, 'static>>; 2] = [None, None];

        unsafe {
            core::ptr::write_volatile(
                0xE000ED24 as *mut u32,
                core::ptr::read_volatile(0xE000ED24 as *mut u32) | (1 << 18 | 1 << 17 | 1 << 16),
            );
        }
        let sc = c.device.SYSCTL.constrain();
        let mut porta = c.device.GPIO_PORTA_AHB.split(&sc.power_control);
        let portf = c.device.GPIO_PORTF_AHB.split(&sc.power_control);
        let portj = c.device.GPIO_PORTJ_AHB.split(&sc.power_control);
        let portn = c.device.GPIO_PORTN.split(&sc.power_control);

        let uart_rx_pin = porta
            .pa0
            .into_af_open_drain::<gpio::AF1, gpio::PullUp>(&mut porta.control);

        let uart_tx_pin = porta
            .pa1
            .into_af_open_drain::<gpio::AF1, gpio::PullUp>(&mut porta.control);

        let mut clocks = sc.clock_setup;
        clocks.oscillator = Oscillator::Main(
            CrystalFrequency::_25mhz,
            SystemClock::UsePll(PllOutputFrequency::_120mhz),
        );
        let clocks = clocks.freeze();

        unsafe {
            SERIAL = Some(tm4c129x_hal::serial::Serial::uart0(
                c.device.UART0,
                uart_tx_pin,
                uart_rx_pin,
                (),
                (),
                115_200.bps(),
                tm4c129x_hal::serial::NewlineMode::SwapLFtoCRLF,
                &clocks,
                &sc.power_control,
            ));
        }

        println!(
            "Chip information: {:#?}",
            tm4c129x_hal::sysctl::chip_id::get()
        );

        let hib = tm4c129x_hal::hib::Hib::hib(
            c.device.HIB,
            tm4c129x_hal::hib::Source::LowFrequencyInternalOscillator,
            &sc.power_control,
        );

        let mut leds: [&mut embedded_hal::digital::OutputPin; 4] = [
            &mut portn.pn1.into_push_pull_output(),
            &mut portn.pn0.into_push_pull_output(),
            &mut portf.pf4.into_push_pull_output(),
            &mut portf.pf0.into_push_pull_output(),
        ];
        let _button1 = portj.pj0.into_pull_up_input();
        let _button2 = portj.pj1.into_pull_up_input();

        for led in &mut leds {
            led.set_high();
            for _ in 0..500_000 {
                cortex_m::asm::nop();
            }
            led.set_low();
        }

        let ephy = tm4c129x_hal::ephy::Peripherals::take().unwrap().EPHY.0;

        let device = EthernetDevice::new(&sc.power_control, clocks, c.device.EMAC0, ephy);

        let neighbor_cache = NeighborCache::new(&mut NEIGHBOR_CACHE_ENTRIES[..]);

        *IP_ADDRS = Some([
            IpCidr::new(IpAddress::v6(0x2001, 0xdb8, 0, 0, 0, 0, 0, 0x2), 64),
            IpCidr::new(IpAddress::v4(192, 0, 2, 2), 24),
        ]);

        let iface = EthernetInterfaceBuilder::new(device)
            .ethernet_addr(EthernetAddress([0x00u8, 0x1A, 0xB6, 0x00, 0x02, 0x74]))
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut IP_ADDRS.as_mut().unwrap()[..])
            .finalize();

        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the
        // data statically to verify that it fits into RAM rather than get
        // undefined behavior when stack overflows.
        // let server_socket = TcpSocket::new(
        //     TcpSocketBuffer::new(&mut TCP_SERVER_RX_DATA[..]),
        //     TcpSocketBuffer::new(&mut TCP_SERVER_TX_DATA[..]),
        // );
        let client_socket = TcpSocket::new(
            TcpSocketBuffer::new(&mut TCP_CLIENT_RX_DATA[..]),
            TcpSocketBuffer::new(&mut TCP_CLIENT_TX_DATA[..]),
        );

        let mut socket_set = SocketSet::new(&mut SOCKET_SET_ENTRIES[..]);
        // let server_handle = socket_set.add(server_socket);
        let client_handle = socket_set.add(client_socket);

        c.spawn.packet_timer(0).unwrap();

        init::LateResources {
            IFACE: iface,
            SOCKET_SET: socket_set,
            // SERVER_HANDLE: server_handle,
            CLIENT_HANDLE: client_handle,
            HIB: hib,
        }
    }

    #[idle(resources = [IFACE, SOCKET_SET, HIB])]
    fn idle(c: idle::Context) -> ! {
        let mut hib = c.resources.HIB;
        let mut socket_set = c.resources.SOCKET_SET;
        let iface = c.resources.IFACE;

        loop {
            let _ = socket_set.lock(|socket_set| {
                iface.poll(
                    socket_set,
                    Instant::from_millis(hib.lock(|hib| hib.get_millis())),
                )
            });

            // if let Some(delay) = c.resources.IFACE.poll_delay(
            //     &*c.resources.SOCKET_SET,
            //     rtfm::Instant::now().to_smol_instant(),
            // ) {
            //     let wake_at = rtfm::Instant::now().to_smol_instant() + delay;
            //     let _ = rtfm::Instant::now().handle_ethernet(wake_at.to_rtfm_instant());
            // } else {
            //     dbg!();
            // }
        }
    }

    #[task(
        schedule = [packet_timer],
        resources=[SOCKET_SET, CLIENT_HANDLE, HIB],
    )]
    fn packet_timer(mut c: packet_timer::Context, i: i64) {
        static mut DID_CONNECT: bool = false;

        if c.resources.HIB.get_millis() > i * 1000 {
            let mut socket = c
                .resources
                .SOCKET_SET
                .get::<TcpSocket>(*c.resources.CLIENT_HANDLE);
            if !socket.is_open() {
                if !*DID_CONNECT {
                    socket
                        .connect(
                            (IpAddress::v6(0x2001, 0xdb8, 0, 0, 0, 0, 0, 1), 49280),
                            (IpAddress::Unspecified, 65000),
                        )
                        .unwrap();
                    *DID_CONNECT = true;
                }
            }

            if socket.can_send() {
                if i % 2 == 0 {
                    socket.send_slice(b"SET channel0 \"ON\"\r").unwrap();
                } else {
                    socket.send_slice(b"SET channel0 \"OFF\"\r").unwrap();
                }
            }

            if socket.can_recv() {
                println!("got {:?}", socket.recv(|buffer| (buffer.len(), buffer)));
            }

            c.schedule
                .packet_timer(c.scheduled + (10000000).cycles(), i + 1)
                .unwrap();
        } else {
            c.schedule
                .packet_timer(c.scheduled + (10000000).cycles(), i)
                .unwrap();
        }
    }

    extern "C" {
        fn UART7();
    }
};
