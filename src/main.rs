#![no_std]
#![no_main]

use core::fmt::Write;
use core::panic::PanicInfo;
use cortex_m_rt::{entry, exception};
use smoltcp::iface::{EthernetInterfaceBuilder, NeighborCache};
use smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use smoltcp::time::{Duration, Instant};
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr};
use tm4c129x;
use tm4c129x_hal::ethernet::EthernetDevice;
use tm4c129x_hal::gpio;
use tm4c129x_hal::prelude::*;
use tm4c129x_hal::sysctl::Clocks;
use tm4c129x_hal::sysctl::SysctlExt;
use tm4c129x_hal::sysctl::{CrystalFrequency, Oscillator, PllOutputFrequency, SystemClock};

#[no_mangle]
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

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    loop {
        cortex_m::asm::wfi();
    }
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

tm4c129x::interrupt!(EMAC0, emac0);
fn emac0() {
    unsafe {
        let emac0 = tm4c129x::Peripherals::steal().EMAC0;
        emac0.dmaris.write(|w| w.bits(0xffff_ffff));
        emac0.ephymisc.write(|w| w.int().set_bit());
    }
}

#[exception]
fn SysTick() {}

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

pub struct Clock {
    systick: tm4c129x::SYST,
    ticks_per_10ms: u64,
    instant: Instant,
}

impl Clock {
    pub fn new(mut systick: tm4c129x::SYST, _clocks: Clocks) -> Clock {
        systick.set_reload(100);
        systick.clear_current();

        let ticks_per_10ms = cortex_m::peripheral::SYST::get_ticks_per_10ms().into();
        assert!(ticks_per_10ms != 0);

        Clock {
            systick,
            ticks_per_10ms,
            instant: Instant::from_millis(0),
        }
    }

    pub fn advance(&mut self, duration: Duration) {
        let reload = (self.ticks_per_10ms * duration.total_millis()) / 10;
        if reload == 0 {
            return;
        }

        let this_sleep = core::cmp::min(reload, 0x00ffffff) as u32;

        self.systick.set_reload(this_sleep);
        self.systick.clear_current();

        self.systick.enable_interrupt();
        self.systick.enable_counter();
        cortex_m::asm::wfe();
        self.systick.disable_counter();

        self.instant += Duration::from_millis((this_sleep as u64 * 10) / self.ticks_per_10ms);
    }

    pub fn elapsed(&mut self) -> Instant {
        self.instant
    }
}

#[entry]
fn main() -> ! {
    let p = tm4c129x_hal::Peripherals::take().unwrap();
    let mut core_p = tm4c129x_hal::CorePeripherals::take().unwrap();
    let sc = p.SYSCTL.constrain();
    let mut porta = p.GPIO_PORTA_AHB.split(&sc.power_control);
    let portf = p.GPIO_PORTF_AHB.split(&sc.power_control);
    let portj = p.GPIO_PORTJ_AHB.split(&sc.power_control);
    let portn = p.GPIO_PORTN.split(&sc.power_control);

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
            p.UART0,
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

    let mut clock = Clock::new(core_p.SYST, clocks);
    let device = EthernetDevice::new(&sc.power_control, clocks, &mut core_p.NVIC, p.EMAC0, ephy);
    let mut neighbor_cache_entries = [None; 8];
    let neighbor_cache = NeighborCache::new(&mut neighbor_cache_entries[..]);

    let mut ip_addrs = [IpCidr::new(IpAddress::v4(10, 5, 2, 2), 8)];
    let mut iface = EthernetInterfaceBuilder::new(device)
        .ethernet_addr(EthernetAddress([0x00u8, 0x1A, 0xB6, 0x00, 0x02, 0x74]))
        .neighbor_cache(neighbor_cache)
        .ip_addrs(&mut ip_addrs[..])
        .finalize();

    // It is not strictly necessary to use a `static mut` and unsafe code here, but
    // on embedded systems that smoltcp targets it is far better to allocate the
    // data statically to verify that it fits into RAM rather than get
    // undefined behavior when stack overflows.
    let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(unsafe {
            static mut TCP_SERVER_RX_DATA: [u8; 1024] = [0; 1024];
            &mut TCP_SERVER_RX_DATA[..]
        }),
        TcpSocketBuffer::new(unsafe {
            static mut TCP_SERVER_TX_DATA: [u8; 1024] = [0; 1024];
            &mut TCP_SERVER_TX_DATA[..]
        }),
    );
    let client_socket = TcpSocket::new(
        TcpSocketBuffer::new(unsafe {
            static mut TCP_CLIENT_RX_DATA: [u8; 1024] = [0; 1024];
            &mut TCP_CLIENT_RX_DATA[..]
        }),
        TcpSocketBuffer::new(unsafe {
            static mut TCP_CLIENT_TX_DATA: [u8; 1024] = [0; 1024];
            &mut TCP_CLIENT_TX_DATA[..]
        }),
    );

    let mut socket_set_entries: [_; 2] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let server_handle = socket_set.add(server_socket);
    let client_handle = socket_set.add(client_socket);

    let mut did_listen = false;
    let mut did_connect = false;

    loop {
        while iface.poll(&mut socket_set, clock.elapsed()).is_err() {}

        {
            let mut socket = socket_set.get::<TcpSocket>(server_handle);
            if !socket.is_active() && !socket.is_listening() {
                if !did_listen {
                    println!("listening");
                    socket.listen(7).unwrap();
                    did_listen = true;
                }
            }

            if socket.can_send() {
                let mut data: [u8; 256] = [0; 256];
                match socket.recv_slice(&mut data) {
                    Ok(0) => {}
                    Ok(count) => {
                        socket.send_slice(&data[0..count]).unwrap();
                    }
                    Err(_) => {}
                }
            }
        }

        {
            let mut socket = socket_set.get::<TcpSocket>(client_handle);
            if !socket.is_open() {
                if !did_connect {
                    socket
                        .connect(
                            (IpAddress::v4(10, 5, 2, 1), 80),
                            (IpAddress::Unspecified, 65000),
                        )
                        .unwrap();
                    did_connect = true;
                }
            }

            if socket.can_send() {
                socket.send_slice(b"GET / HTTP/1.1\r\n\r\n").unwrap();
                socket.close();
            }

            if socket.can_recv() {
                println!("got {:?}", socket.recv(|buffer| (buffer.len(), buffer)));
            }
        }

        let delay = iface
            .poll_delay(&socket_set, clock.elapsed())
            .unwrap_or(Duration::from_millis(0xffff_ffff_ffff_ffff));
        clock.advance(delay);
    }
}
