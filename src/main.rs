#![no_std]
#![no_main]

extern crate panic_semihosting;

use bare_metal::Nr;
use core::fmt::Write;
use cortex_m_rt::{entry, exception};
use smoltcp::iface::{EthernetInterfaceBuilder, NeighborCache};
use smoltcp::phy::{ChecksumCapabilities, Device, DeviceCapabilities, RxToken, TxToken};
use smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use smoltcp::time::{Duration, Instant};
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr};
use tivaware_sys::*;
use tm4c129x;
use tm4c129x_hal::gpio;
use tm4c129x_hal::prelude::*;
use tm4c129x_hal::sysctl::SysctlExt;
use tm4c129x_hal::sysctl::{CrystalFrequency, Oscillator, PllOutputFrequency, SystemClock};
use vcell::VolatileCell;

macro_rules! println {
    ($fmt:expr) => {
        #[allow(unused_unsafe)]
        unsafe {
            let _ = writeln!(SERIAL.as_mut().unwrap(), $fmt);
        }
    };
    ($fmt:expr, $($arg:tt)*) => {
        #[allow(unused_unsafe)]
        unsafe {
            let _ = writeln!(SERIAL.as_mut().unwrap(), $fmt, $($arg)*);
        }
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

struct Tm4cEthernetDevice {
    tx_index: usize,
    rx_index: usize,
}

impl Tm4cEthernetDevice {
    fn new(
        lock: &tm4c129x_hal::sysctl::PowerControl,
        clocks: tm4c129x_hal::sysctl::Clocks,
    ) -> Tm4cEthernetDevice {
        unsafe {
            let emac0_base: u32 = tm4c129x::EMAC0::ptr() as u32;

            tm4c129x_hal::sysctl::control_power(
                lock,
                tm4c129x_hal::sysctl::Domain::Emac0,
                tm4c129x_hal::sysctl::RunMode::Run,
                tm4c129x_hal::sysctl::PowerState::On,
            );
            tm4c129x_hal::sysctl::control_power(
                lock,
                tm4c129x_hal::sysctl::Domain::Ephy0,
                tm4c129x_hal::sysctl::RunMode::Run,
                tm4c129x_hal::sysctl::PowerState::On,
            );
            tm4c129x_hal::sysctl::reset(lock, tm4c129x_hal::sysctl::Domain::Emac0);
            tm4c129x_hal::sysctl::reset(lock, tm4c129x_hal::sysctl::Domain::Ephy0);

            EMACReset(emac0_base);

            EMACPHYConfigSet(
                emac0_base,
                EMAC_PHY_TYPE_INTERNAL | EMAC_PHY_INT_MDIX_EN | EMAC_PHY_AN_100B_T_FULL_DUPLEX,
            );

            EMACInit(
                emac0_base,
                clocks.sysclk.0,
                EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED,
                4,
                4,
                1,
            );

            EMACConfigSet(
                emac0_base,
                EMAC_CONFIG_FULL_DUPLEX
                    | EMAC_CONFIG_CHECKSUM_OFFLOAD
                    | EMAC_CONFIG_7BYTE_PREAMBLE
                    | EMAC_CONFIG_IF_GAP_96BITS
                    | EMAC_CONFIG_USE_MACADDR0
                    | EMAC_CONFIG_SA_FROM_DESCRIPTOR
                    | EMAC_CONFIG_BO_LIMIT_1024,
                EMAC_MODE_RX_STORE_FORWARD
                    | EMAC_MODE_TX_STORE_FORWARD
                    | EMAC_MODE_TX_THRESHOLD_64_BYTES
                    | EMAC_MODE_RX_THRESHOLD_64_BYTES,
                0,
            );

            for i in 0..NUM_TX_DESCRIPTORS {
                TX_DESCRIPTORS[i].buffer_len = DES1_TX_CTRL_SADDR_INSERT;
                TX_DESCRIPTORS[i].link_or_buffer_ext.link = if i == NUM_TX_DESCRIPTORS - 1 {
                    &mut TX_DESCRIPTORS[0] as *mut _
                } else {
                    &mut TX_DESCRIPTORS[i + 1] as *mut _
                };
                TX_DESCRIPTORS[i].ctrl_status.set(
                    DES0_TX_CTRL_LAST_SEG
                        | DES0_TX_CTRL_FIRST_SEG
                        | DES0_TX_CTRL_INTERRUPT
                        | DES0_TX_CTRL_CHAINED
                        | DES0_TX_CTRL_IP_ALL_CKHSUMS,
                );
            }

            for i in 0..NUM_RX_DESCRIPTORS {
                RX_DESCRIPTORS[i].ctrl_status.set(DES0_RX_CTRL_OWN);
                RX_DESCRIPTORS[i].buffer_len =
                    DES1_RX_CTRL_CHAINED | ((RX_BUFFER_SIZE as u32) << DES1_RX_CTRL_BUFF1_SIZE_S);
                RX_DESCRIPTORS[i].buffer = &mut RX_BUFFERS[i][0] as *mut u8 as *mut _;
                RX_DESCRIPTORS[i].link_or_buffer_ext.link = if i == (NUM_RX_DESCRIPTORS - 1) {
                    &mut RX_DESCRIPTORS[0] as *mut _
                } else {
                    &mut RX_DESCRIPTORS[i + 1] as *mut _
                };
            }

            EMACRxDMADescriptorListSet(emac0_base, &mut RX_DESCRIPTORS as *mut _ as *mut _);
            EMACTxDMADescriptorListSet(emac0_base, &mut TX_DESCRIPTORS as *mut _ as *mut _);

            EMACAddrSet(
                emac0_base,
                0,
                &[0x00u8, 0x1A, 0xB6, 0x00, 0x02, 0x74] as *const _,
            );

            while EMACPHYRead(emac0_base, 0, EPHY_BMSR as u8) as u32 & EPHY_BMSR_LINKSTAT == 0 {
                cortex_m::asm::nop();
            }

            EMACFrameFilterSet(
                emac0_base,
                EMAC_FRMFILTER_RX_ALL | EMAC_FRMFILTER_PROMISCUOUS,
            );

            EMACIntClear(emac0_base, EMACIntStatus(emac0_base, false));
            EMACTxEnable(emac0_base);
            EMACRxEnable(emac0_base);
            IntEnable(tm4c129x::Interrupt::EMAC0.nr() as u32 + 16);
        }

        Tm4cEthernetDevice {
            rx_index: 0,
            tx_index: 0,
        }
    }
}

impl<'a> Device<'a> for Tm4cEthernetDevice {
    type RxToken = Tm4cRxToken<'a>;
    type TxToken = Tm4cTxToken<'a>;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        // Make sure that we own the receive descriptor.
        unsafe {
            if RX_DESCRIPTORS[self.rx_index].ctrl_status.get() & DES0_RX_CTRL_OWN
                == DES0_RX_CTRL_OWN
            {
                return None;
            }

            if TX_DESCRIPTORS[self.tx_index].ctrl_status.get() & DES0_TX_CTRL_OWN
                == DES0_TX_CTRL_OWN
            {
                println!("receive no tx buffers");
                return None;
            }
        }

        println!("receive got both buffers");
        let result = Some((
            Tm4cRxToken {
                descriptor: unsafe { &mut RX_DESCRIPTORS[self.rx_index] },
            },
            Tm4cTxToken {
                descriptor: unsafe { &mut TX_DESCRIPTORS[self.tx_index] },
            },
        ));

        self.tx_index += 1;
        if self.tx_index == NUM_TX_DESCRIPTORS {
            self.tx_index = 0;
        }

        self.rx_index += 1;
        if self.rx_index == NUM_RX_DESCRIPTORS {
            self.rx_index = 0;
        }

        result
    }

    fn transmit(&'a mut self) -> Option<(Self::TxToken)> {
        println!("transmit");

        unsafe {
            if TX_DESCRIPTORS[self.tx_index].ctrl_status.get() & DES0_TX_CTRL_OWN
                == DES0_TX_CTRL_OWN
            {
                println!("tx no buffer");
                return None;
            }
        }

        let result = Some(Tm4cTxToken {
            descriptor: unsafe { &mut TX_DESCRIPTORS[self.tx_index] },
        });

        self.tx_index += 1;
        if self.tx_index == NUM_TX_DESCRIPTORS {
            self.tx_index = 0;
        }

        println!("tx returning");

        result
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut cap = DeviceCapabilities::default();

        cap.max_transmission_unit = RX_BUFFER_SIZE;
        cap.max_burst_size = Some(1);
        cap.checksum = ChecksumCapabilities::ignored();

        cap
    }
}

struct Tm4cRxToken<'a> {
    descriptor: &'a mut emac_descriptor,
}

impl<'a> RxToken for Tm4cRxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&[u8]) -> smoltcp::Result<R>,
    {
        println!("rx start consume");
        let result;

        unsafe {
            // We own the receive descriptor so check to see if it contains a valid frame.
            if self.descriptor.ctrl_status.get() & DES0_RX_STAT_ERR != DES0_RX_STAT_ERR {
                // We have a valid frame. First check that the "last descriptor" flag is set. We
                // sized the receive buffer such that it can always hold a valid frame so this
                // flag should never be clear at this point but...
                if self.descriptor.ctrl_status.get() & DES0_RX_STAT_LAST_DESC
                    == DES0_RX_STAT_LAST_DESC
                {
                    // What size is the received frame?
                    let frame_len = (self.descriptor.ctrl_status.get()
                        & DES0_RX_STAT_FRAME_LENGTH_M)
                        >> DES0_RX_STAT_FRAME_LENGTH_S;
                    let data = core::slice::from_raw_parts(
                        self.descriptor.buffer as *mut u8,
                        frame_len as usize,
                    );
                    println!("RX: {:?}", data);
                    result = f(data);
                    println!("rx processed");
                } else {
                    println!("rx truncated");
                    result = Err(smoltcp::Error::Truncated);
                }
            } else {
                println!("rx checksum");
                result = Err(smoltcp::Error::Checksum);
            }
            self.descriptor.ctrl_status.set(DES0_RX_CTRL_OWN);
        }
        println!("rx end consume");

        result
    }
}

struct Tm4cTxToken<'a> {
    descriptor: &'a mut emac_descriptor,
}

impl<'a> TxToken for Tm4cTxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        println!("tx start consume");
        let emac0_base: u32 = tm4c129x::EMAC0::ptr() as u32;
        let result;

        assert!(len <= RX_BUFFER_SIZE);

        unsafe {
            let mut data: [u8; RX_BUFFER_SIZE] = core::mem::uninitialized();
            result = f(&mut data);

            // Fill in the packet size and pointer, and tell the transmitter to start work.
            self.descriptor.buffer_len = len as u32;
            self.descriptor.buffer = &mut data as *mut _ as *mut _;
            self.descriptor.ctrl_status.set(
                DES0_TX_CTRL_LAST_SEG
                    | DES0_TX_CTRL_FIRST_SEG
                    | DES0_TX_CTRL_INTERRUPT
                    | DES0_TX_CTRL_CHAINED
                    | DES0_TX_CTRL_OWN,
            );

            // Tell the DMA to reacquire the descriptor now that we've filled it in. This
            // call is benign if the transmitter hasn't stalled and checking the state takes
            // longer than just issuing a poll demand so we do this for all packets.
            EMACTxDMAPollDemand(emac0_base);
        }

        println!("tx end consume");

        result
    }
}

mod mock {
    use core::cell::Cell;
    use smoltcp::time::{Duration, Instant};

    #[derive(Debug)]
    pub struct Clock(Cell<Instant>);

    impl Clock {
        pub fn new() -> Clock {
            Clock(Cell::new(Instant::from_millis(0)))
        }

        pub fn advance(&self, duration: Duration) {
            self.0.set(self.0.get() + duration)
        }

        pub fn elapsed(&self) -> Instant {
            self.0.get()
        }
    }
}

#[repr(C)]
union emac_des3 {
    link: *mut emac_descriptor,
    buffer_ext: *mut cty::c_void,
}

#[repr(C)]
struct emac_descriptor {
    ctrl_status: VolatileCell<u32>,
    buffer_len: u32,
    buffer: *mut cty::c_void,
    link_or_buffer_ext: emac_des3,
    ext_rx_status: u32,
    reserved: u32,
    ieee_1588_time_lo: u32,
    ieee_1588_time_hi: u32,
}

const fn d() -> emac_descriptor {
    emac_descriptor {
        ctrl_status: VolatileCell::new(0),
        buffer_len: 0,
        buffer: core::ptr::null_mut(),
        link_or_buffer_ext: emac_des3 {
            link: core::ptr::null_mut(),
        },
        ext_rx_status: 0,
        reserved: 0,
        ieee_1588_time_lo: 0,
        ieee_1588_time_hi: 0,
    }
}

const NUM_TX_DESCRIPTORS: usize = 10;
const NUM_RX_DESCRIPTORS: usize = 10;
static mut RX_DESCRIPTORS: [emac_descriptor; NUM_TX_DESCRIPTORS] =
    [d(), d(), d(), d(), d(), d(), d(), d(), d(), d()];
static mut TX_DESCRIPTORS: [emac_descriptor; NUM_RX_DESCRIPTORS] =
    [d(), d(), d(), d(), d(), d(), d(), d(), d(), d()];

const RX_BUFFER_SIZE: usize = 1536;
static mut RX_BUFFERS: [[u8; RX_BUFFER_SIZE]; NUM_RX_DESCRIPTORS] =
    [[0; RX_BUFFER_SIZE]; NUM_RX_DESCRIPTORS];

#[entry]
fn main() -> ! {
    let p = tm4c129x_hal::Peripherals::take().unwrap();
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

    let clock = mock::Clock::new();
    let device = Tm4cEthernetDevice::new(&sc.power_control, clocks);
    let mut neighbor_cache_entries = [None; 8];
    let neighbor_cache = NeighborCache::new(&mut neighbor_cache_entries[..]);

    let mut ip_addrs = [IpCidr::new(IpAddress::v4(10, 5, 2, 2), 8)];
    let mut iface = EthernetInterfaceBuilder::new(device)
        .ethernet_addr(EthernetAddress::default())
        .neighbor_cache(neighbor_cache)
        .ip_addrs(&mut ip_addrs[..])
        .finalize();

    let server_socket = {
        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the
        // data statically to verify that it fits into RAM rather than get
        // undefined behavior when stack overflows.
        static mut TCP_SERVER_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_SERVER_TX_DATA: [u8; 1024] = [0; 1024];
        let tcp_rx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_RX_DATA[..] });
        let tcp_tx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_TX_DATA[..] });
        TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)
    };

    let client_socket = {
        static mut TCP_CLIENT_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_CLIENT_TX_DATA: [u8; 1024] = [0; 1024];
        let tcp_rx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_CLIENT_RX_DATA[..] });
        let tcp_tx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_CLIENT_TX_DATA[..] });
        TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)
    };

    let mut socket_set_entries: [_; 2] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let server_handle = socket_set.add(server_socket);
    let client_handle = socket_set.add(client_socket);

    let mut did_listen = false;
    let mut did_connect = false;

    loop {
        iface.poll(&mut socket_set, clock.elapsed()).unwrap();

        {
            let mut socket = socket_set.get::<TcpSocket>(server_handle);
            if !socket.is_active() && !socket.is_listening() {
                if !did_listen {
                    println!("listening");
                    socket.listen(1234).unwrap();
                    did_listen = true;
                }
            }

            if socket.can_recv() {
                println!("can rx");
                println!("got {:?}", socket.recv(|buffer| (buffer.len(), buffer)));
            }
        }

        {
            let mut socket = socket_set.get::<TcpSocket>(client_handle);
            if !socket.is_open() {
                if !did_connect {
                    println!("connecting");
                    socket
                        .connect(
                            (IpAddress::v4(127, 0, 0, 1), 1234),
                            (IpAddress::Unspecified, 65000),
                        )
                        .unwrap();
                    println!("after connect");
                    did_connect = true;
                }
            }

            if socket.can_send() {
                println!("sending");
                socket.send_slice(b"0123456789abcdef").unwrap();
                socket.close();
            }
        }

        match iface.poll_delay(&socket_set, clock.elapsed()) {
            Some(Duration { millis: 0 }) => {}
            Some(delay) => {
                println!("sleeping for {} ms", delay);
                clock.advance(delay)
            }
            None => clock.advance(Duration::from_millis(1)),
        }
    }
}
