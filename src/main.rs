#![no_std]
#![no_main]
#![allow(deprecated)]
#![warn(clippy::all, clippy::pedantic)]

use core::fmt::Write;
use core::{convert::TryInto, mem::MaybeUninit, panic::PanicInfo};
use cortex_m_rt::exception;
use heapless::{consts, String, Vec};
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

/// Get port between 49152 and 65535
fn get_ephemeral_port() -> u16 {
    let rand = unsafe { (*tm4c129x::HIB::ptr()).rtcss.read().bits() };

    let mut rand = rand + 49142;
    while rand > 65535 {
        rand -= 65535 - 49142;
    }

    rand.try_into().unwrap()
}

#[derive(Copy, Clone, Debug)]
pub enum SwitchWhen {
    Always,
    SecondariesOn,
    SecondariesOff,
    Never,
}

pub struct Zone {
    name: &'static str,
    mixer_channel: u8,
    // relay: gpio::Pxx<gpio::Output<gpio::PushPull>>,
}

pub type ChannelStatus = (
    gpio::Pxx<gpio::Input<gpio::PullUp>>,
    Vec<(&'static Zone, SwitchWhen), consts::U8>,
);

pub struct ControlStation {
    name: &'static str,
    mixer_channel: u8,
    buttons: Vec<ChannelStatus, consts::U8>,
}

macro_rules! vec {
    () => (Vec::new());
    ($($x:expr),*) => ({let mut v = Vec::new(); $(v.push($x).map_err(|_| ()).unwrap();)* v});
    ($($x:expr,)*) => (vec![$($x),*]);
}

#[rtic::app(device = tm4c129x_hal::tm4c129x, peripherals = true, monotonic=rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        IFACE: EthernetInterface<'static, 'static, 'static, EthernetDevice>,

        SOCKET_SET: SocketSet<'static, 'static, 'static>,
        CLIENT_HANDLE: SocketHandle,

        HIB: tm4c129x_hal::hib::Hib,

        CONTROL_STATIONS: [ControlStation; 5],
        SECONDARIES_DETECT: gpio::Pxx<gpio::Input<gpio::PullUp>>,
    }

    #[init(spawn = [gpio_poll, keepalive])]
    fn init(mut c: init::Context) -> init::LateResources {
        static mut TCP_CLIENT_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_CLIENT_TX_DATA: [u8; 1024] = [0; 1024];

        static mut IP_ADDRS: Option<[IpCidr; 1]> = None;
        static mut NEIGHBOR_CACHE_ENTRIES: [Option<(IpAddress, Neighbor)>; 8] = [None; 8];
        static mut SOCKET_SET_ENTRIES: [Option<SocketSetItem<'static, 'static>>; 2] = [None, None];

        let sc = c.device.SYSCTL.constrain();
        let porta = c.device.GPIO_PORTA_AHB.split(&sc.power_control);
        // let portb = c.device.GPIO_PORTB_AHB.split(&sc.power_control);
        // let portc = c.device.GPIO_PORTC_AHB.split(&sc.power_control);
        let portd = c.device.GPIO_PORTD_AHB.split(&sc.power_control);
        // let porte = c.device.GPIO_PORTE_AHB.split(&sc.power_control);
        let portf = c.device.GPIO_PORTF_AHB.split(&sc.power_control);
        let portg = c.device.GPIO_PORTG_AHB.split(&sc.power_control);
        let porth = c.device.GPIO_PORTH_AHB.split(&sc.power_control);
        // let portj = c.device.GPIO_PORTJ_AHB.split(&sc.power_control);
        let portk = c.device.GPIO_PORTK.split(&sc.power_control);
        // let portl = c.device.GPIO_PORTL.split(&sc.power_control);
        let portm = c.device.GPIO_PORTM.split(&sc.power_control);
        let portn = c.device.GPIO_PORTN.split(&sc.power_control);
        let portp = c.device.GPIO_PORTP.split(&sc.power_control);
        let portq = c.device.GPIO_PORTQ.split(&sc.power_control);

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
            cortex_m::asm::delay(500_000);
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
            .ethernet_addr(EthernetAddress([0x00, 0x1A, 0xB6, 0x00, 0x02, 0x74]))
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

        static mut OFFICES: MaybeUninit<Zone> = MaybeUninit::uninit();
        static mut BACKSTAGE: MaybeUninit<Zone> = MaybeUninit::uninit();
        static mut STAGE: MaybeUninit<Zone> = MaybeUninit::uninit();
        static mut FRONT_OF_HOUSE: MaybeUninit<Zone> = MaybeUninit::uninit();
        static mut REHEARSAL_ROOM: MaybeUninit<Zone> = MaybeUninit::uninit();

        let offices = unsafe {
            OFFICES.as_mut_ptr().write(Zone {
                name: "Offices",
                mixer_channel: 1,
                // relay: portn.pn5.into_push_pull_output().downgrade().downgrade(),
            });
            &*OFFICES.as_ptr()
        };
        let backstage = unsafe {
            BACKSTAGE.as_mut_ptr().write(Zone {
                name: "Backstage",
                mixer_channel: 2,
                // relay: portn.pn4.into_push_pull_output().downgrade().downgrade(),
            });
            &*BACKSTAGE.as_ptr()
        };
        let stage = unsafe {
            STAGE.as_mut_ptr().write(Zone {
                name: "Stage and Auditorium",
                mixer_channel: 3,
                // relay: portp.pp4.into_push_pull_output().downgrade().downgrade(),
            });
            &*STAGE.as_ptr()
        };
        let front_of_house = unsafe {
            FRONT_OF_HOUSE.as_mut_ptr().write(Zone {
                name: "Front of House",
                mixer_channel: 4,
                // relay: portq.pq0.into_push_pull_output().downgrade().downgrade(),
            });
            &*FRONT_OF_HOUSE.as_ptr()
        };
        let rehearsal_room = unsafe {
            REHEARSAL_ROOM.as_mut_ptr().write(Zone {
                name: "Rehearsal Room",
                mixer_channel: 5,
                // relay: portd.pd5.into_push_pull_output().downgrade().downgrade(),
            });
            &*REHEARSAL_ROOM.as_ptr()
        };

        let control_stations = [
            ControlStation {
                name: "Management",
                mixer_channel: 2,
                buttons: vec![
                    (
                        portm.pm6.into_pull_up_input().downgrade().downgrade(),
                        vec![(offices, SwitchWhen::Always)],
                    ),
                    (
                        porth.ph0.into_pull_up_input().downgrade().downgrade(),
                        vec![(backstage, SwitchWhen::Always)],
                    ),
                    (
                        porth.ph1.into_pull_up_input().downgrade().downgrade(),
                        vec![(stage, SwitchWhen::Always)],
                    ),
                    (
                        portk.pk6.into_pull_up_input().downgrade().downgrade(),
                        vec![(front_of_house, SwitchWhen::Always)],
                    ),
                    (
                        portk.pk7.into_pull_up_input().downgrade().downgrade(),
                        vec![(rehearsal_room, SwitchWhen::Always)],
                    ),
                ],
            },
            ControlStation {
                name: "SM",
                mixer_channel: 3,
                buttons: vec![
                    (
                        porta.pa7.into_pull_up_input().downgrade().downgrade(),
                        vec![(offices, SwitchWhen::Never)],
                    ),
                    (
                        portq.pq2.into_pull_up_input().downgrade().downgrade(),
                        vec![(backstage, SwitchWhen::Always)],
                    ),
                    (
                        portq.pq3.into_pull_up_input().downgrade().downgrade(),
                        vec![(stage, SwitchWhen::Never)],
                    ),
                    (
                        portp.pp3.into_pull_up_input().downgrade().downgrade(),
                        vec![(front_of_house, SwitchWhen::Always)],
                    ),
                    (
                        portq.pq1.into_pull_up_input().downgrade().downgrade(),
                        vec![(rehearsal_room, SwitchWhen::Never)],
                    ),
                ],
            },
            ControlStation {
                name: "Larkum",
                mixer_channel: 4,
                buttons: vec![
                    (
                        portn.pn2.into_pull_up_input().downgrade().downgrade(),
                        vec![(offices, SwitchWhen::Always)],
                    ),
                    (
                        portn.pn3.into_pull_up_input().downgrade().downgrade(),
                        vec![(backstage, SwitchWhen::Always)],
                    ),
                    (
                        portp.pp2.into_pull_up_input().downgrade().downgrade(),
                        vec![(stage, SwitchWhen::Always)],
                    ),
                    (
                        portm.pm7.into_pull_up_input().downgrade().downgrade(),
                        vec![(front_of_house, SwitchWhen::Always)],
                    ),
                    (
                        portp.pp5.into_pull_up_input().downgrade().downgrade(),
                        vec![(rehearsal_room, SwitchWhen::Always)],
                    ),
                ],
            },
            ControlStation {
                name: "Test",
                mixer_channel: 5,
                buttons: vec![
                    (
                        portm.pm3.into_pull_up_input().downgrade().downgrade(),
                        vec![(offices, SwitchWhen::Always)],
                    ),
                    (
                        porth.ph2.into_pull_up_input().downgrade().downgrade(),
                        vec![(backstage, SwitchWhen::Always)],
                    ),
                    (
                        porth.ph3.into_pull_up_input().downgrade().downgrade(),
                        vec![(stage, SwitchWhen::Always)],
                    ),
                    (
                        portd.pd1.into_pull_up_input().downgrade().downgrade(),
                        vec![(front_of_house, SwitchWhen::Always)],
                    ),
                ],
            },
            ControlStation {
                name: "Phone",
                mixer_channel: 1,
                buttons: vec![(
                    portf.pf2.into_pull_up_input().downgrade().downgrade(),
                    vec![
                        (offices, SwitchWhen::Always),
                        (backstage, SwitchWhen::Always),
                        (stage, SwitchWhen::SecondariesOff),
                        (front_of_house, SwitchWhen::SecondariesOff),
                        (rehearsal_room, SwitchWhen::Always),
                    ],
                )],
            },
        ];

        let secondaries_detect = portg.pg0.into_pull_up_input().downgrade().downgrade();

        log::info!("{}: boot", hib.get_millis());

        init::LateResources {
            IFACE: iface,
            SOCKET_SET: socket_set,
            CLIENT_HANDLE: client_handle,
            HIB: hib,
            CONTROL_STATIONS: control_stations,
            SECONDARIES_DETECT: secondaries_detect,
        }
    }

    #[idle(resources = [IFACE, SOCKET_SET, HIB])]
    fn idle(c: idle::Context) -> ! {
        let mut hib = c.resources.HIB;
        let mut socket_set = c.resources.SOCKET_SET;
        let iface = c.resources.IFACE;

        loop {
            let now: i64 = hib.lock(|hib| hib.get_millis().try_into().unwrap());

            socket_set.lock(|socket_set| {
                let _ = iface.poll(socket_set, Instant::from_millis(now));
            });
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

                let send_runmode = |buffer: &mut [u8]| {
                    let bytes = b"devstatus runmode\n";

                    if buffer.len() > bytes.len() {
                        buffer[0..bytes.len()].copy_from_slice(bytes);
                        (bytes.len(), true)
                    } else {
                        (0, false)
                    }
                };

                if let Ok(true) = socket.send(send_runmode) {
                    log::info!("sent keepalive");
                }
            })
        });

        c.schedule
            .keepalive(c.scheduled + Duration::from_cycles(120_000_000))
            .unwrap();
    }

    #[task(
        schedule = [gpio_poll],
        resources=[SOCKET_SET, CLIENT_HANDLE, CONTROL_STATIONS, HIB, SECONDARIES_DETECT],
        priority=2,
    )]
    fn gpio_poll(c: gpio_poll::Context) {
        static mut OLD_STATE: [Option<u8>; 10] = [None; 10];
        let mut new_state: [u8; 10] = [0; 10];

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

        let secondaries_on = c.resources.SECONDARIES_DETECT.is_high();

        for control_station in c.resources.CONTROL_STATIONS.iter() {
            for (button, zones) in control_station.buttons.iter() {
                if button.is_high() {
                    continue;
                }

                for (zone, switch_when) in zones.iter() {
                    match (switch_when, secondaries_on) {
                        (SwitchWhen::Always, _)
                        | (SwitchWhen::SecondariesOn, true)
                        | (SwitchWhen::SecondariesOff, false) => {}
                        _ => continue,
                    }

                    if new_state[usize::from(zone.mixer_channel) - 1] != 0 {
                        continue;
                    }

                    new_state[usize::from(zone.mixer_channel) - 1] = control_station.mixer_channel;
                    new_state[usize::from(zone.mixer_channel) + 5 - 1] = 1;

                    log::info!("{} -> {}", control_station.name, zone.name);
                }
            }
        }

        let mut data = String::<consts::U512>::new();

        let send_update = |buffer: &mut [u8]| {
            for (index, (&old_state, &new_state)) in
                OLD_STATE.iter().zip(new_state.iter()).enumerate()
            {
                if Some(new_state) != old_state {
                    let mut s = String::<consts::U32>::new();

                    writeln!(s, "set MTX::index_{} 0 0 {}", index + 1, new_state).unwrap();

                    data.push_str(s.as_str()).unwrap();
                }
            }

            if buffer.len() > data.len() {
                buffer[0..data.len()].copy_from_slice(data.as_bytes());
                (data.len(), true)
            } else {
                (0, false)
            }
        };

        if let Ok(true) = socket.send(send_update) {
            for (old_state, &new_state) in OLD_STATE.iter_mut().zip(new_state.iter()) {
                *old_state = Some(new_state);
            }

            if data.len() > 0 {
                log::info!("{}: tx: {:?}", c.resources.HIB.get_millis(), data);
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
