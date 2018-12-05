#![no_std]
#![no_main]

extern crate panic_semihosting;
extern crate tivaware_sys;

use bare_metal::Nr;
use core::fmt::Write;
use cortex_m_rt::{entry, exception};
// use cortex_m_semihosting::{hprint, hprintln};
use tivaware_sys::*;
use tm4c129x;
use tm4c129x::interrupt;
use tm4c129x_hal::gpio;
use tm4c129x_hal::prelude::*;
use tm4c129x_hal::sysctl::SysctlExt;
use tm4c129x_hal::sysctl::{CrystalFrequency, Oscillator, PllOutputFrequency, SystemClock};

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    // prints the exception frame as a panic message
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("IRQ: {}", irqn);
}

fn init_ethernet(lock: &tm4c129x_hal::sysctl::PowerControl, clocks: tm4c129x_hal::sysctl::Clocks) {
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
            TX_DESCRIPTORS[i].ui32Count = DES1_TX_CTRL_SADDR_INSERT;
            TX_DESCRIPTORS[i].DES3.pLink = if i == NUM_TX_DESCRIPTORS - 1 {
                &mut TX_DESCRIPTORS[0] as *mut _
            } else {
                &mut TX_DESCRIPTORS[i + 1] as *mut _
            };
            TX_DESCRIPTORS[i].ui32CtrlStatus = DES0_TX_CTRL_LAST_SEG
                | DES0_TX_CTRL_FIRST_SEG
                | DES0_TX_CTRL_INTERRUPT
                | DES0_TX_CTRL_CHAINED
                | DES0_TX_CTRL_IP_ALL_CKHSUMS;
        }

        for i in 0..NUM_RX_DESCRIPTORS {
            RX_DESCRIPTORS[i].ui32CtrlStatus = DES0_RX_CTRL_OWN;
            RX_DESCRIPTORS[i].ui32Count =
                DES1_RX_CTRL_CHAINED | ((RX_BUFFER_SIZE as u32) << DES1_RX_CTRL_BUFF1_SIZE_S);
            RX_DESCRIPTORS[i].pvBuffer1 = &mut RX_BUFFERS[i][0] as *mut u8 as *mut _;
            RX_DESCRIPTORS[i].DES3.pLink = if i == (NUM_RX_DESCRIPTORS - 1) {
                &mut RX_DESCRIPTORS[0] as *mut _
            } else {
                &mut RX_DESCRIPTORS[i + 1] as *mut _
            };
        }

        EMACRxDMADescriptorListSet(emac0_base, &mut RX_DESCRIPTORS as *mut _ as *mut _);
        EMACTxDMADescriptorListSet(emac0_base, &mut TX_DESCRIPTORS as *mut _ as *mut _);
        RX_DESCRIPTOR_INDEX = 0;
        TX_DESCRIPTOR_INDEX = NUM_TX_DESCRIPTORS - 1;

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
        tivaware_sys::EMACTxEnable(emac0_base);
        tivaware_sys::EMACRxEnable(emac0_base);
        tivaware_sys::IntEnable(tm4c129x::Interrupt::EMAC0.nr() as u32 + 16);
        tivaware_sys::EMACIntEnable(emac0_base, tivaware_sys::EMAC_INT_RECEIVE);
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

fn transmit_ethernet(data: &[u8]) {
    let emac0_base: u32 = tm4c129x::EMAC0::ptr() as u32;

    unsafe {
        // Move to the next descriptor.
        TX_DESCRIPTOR_INDEX += 1;
        if TX_DESCRIPTOR_INDEX == NUM_TX_DESCRIPTORS {
            TX_DESCRIPTOR_INDEX = 0;
        }

        // Wait for the transmit descriptor to free up.
        while TX_DESCRIPTORS[TX_DESCRIPTOR_INDEX].ui32CtrlStatus & DES0_TX_CTRL_OWN
            == DES0_TX_CTRL_OWN
        {
            cortex_m::asm::nop();
        }

        // Fill in the packet size and pointer, and tell the transmitter to start work.
        TX_DESCRIPTORS[TX_DESCRIPTOR_INDEX].ui32Count = data.len() as u32;
        TX_DESCRIPTORS[TX_DESCRIPTOR_INDEX].pvBuffer1 = data as *const _ as *mut _;
        TX_DESCRIPTORS[TX_DESCRIPTOR_INDEX].ui32CtrlStatus = DES0_TX_CTRL_LAST_SEG
            | DES0_TX_CTRL_FIRST_SEG
            | DES0_TX_CTRL_INTERRUPT
            | DES0_TX_CTRL_CHAINED
            | DES0_TX_CTRL_OWN;

        // Tell the DMA to reacquire the descriptor now that we've filled it in. This
        // call is benign if the transmitter hasn't stalled and checking the state takes
        // longer than just issuing a poll demand so we do this for all packets.
        EMACTxDMAPollDemand(emac0_base);
    }
}

fn emac_0() {
    let emac0_base: u32 = tm4c129x::EMAC0::ptr() as u32;

    unsafe {
        let int_status = EMACIntStatus(emac0_base, true);
        EMACIntClear(emac0_base, int_status);

        if int_status & EMAC_INT_RECEIVE == EMAC_INT_RECEIVE {
            // Make sure that we own the receive descriptor.
            if RX_DESCRIPTORS[RX_DESCRIPTOR_INDEX].ui32CtrlStatus & DES0_RX_CTRL_OWN
                != DES0_RX_CTRL_OWN
            {
                // We own the receive descriptor so check to see if it contains a valid frame.
                if RX_DESCRIPTORS[RX_DESCRIPTOR_INDEX].ui32CtrlStatus & DES0_RX_STAT_ERR
                    != DES0_RX_STAT_ERR
                {
                    // We have a valid frame. First check that the "last descriptor" flag is set. We
                    // sized the receive buffer such that it can always hold a valid frame so this
                    // flag should never be clear at this point but...
                    if RX_DESCRIPTORS[RX_DESCRIPTOR_INDEX].ui32CtrlStatus & DES0_RX_STAT_LAST_DESC
                        == DES0_RX_STAT_LAST_DESC
                    {
                        // What size is the received frame?
                        let frame_len = (RX_DESCRIPTORS[RX_DESCRIPTOR_INDEX].ui32CtrlStatus
                            & DES0_RX_STAT_FRAME_LENGTH_M)
                            >> DES0_RX_STAT_FRAME_LENGTH_S;

                        let _ = writeln!(
                            SERIAL.as_mut().unwrap(),
                            "RX: {:?}",
                            core::slice::from_raw_parts(
                                RX_DESCRIPTORS[RX_DESCRIPTOR_INDEX].pvBuffer1 as *mut u8,
                                frame_len as usize
                            )
                        );
                    }
                }
                // Now that we are finished dealing with this descriptor, hand it back to the
                // hardware. Note that we assume ApplicationProcessFrame() is finished with the
                // buffer at this point so it is safe to reuse.
                RX_DESCRIPTORS[RX_DESCRIPTOR_INDEX].ui32CtrlStatus = DES0_RX_CTRL_OWN;

                // Move on to the next descriptor in the chain.
                RX_DESCRIPTOR_INDEX += 1;
                if RX_DESCRIPTOR_INDEX == NUM_RX_DESCRIPTORS {
                    RX_DESCRIPTOR_INDEX = 0;
                }
            }
        }
    }
}

const fn d() -> tEMACDMADescriptor {
    tEMACDMADescriptor {
        ui32CtrlStatus: 0,
        ui32Count: 0,
        pvBuffer1: core::ptr::null_mut(),
        DES3: tEMACDES3 {
            pvBuffer2: core::ptr::null_mut(),
        },
        ui32ExtRxStatus: 0,
        ui32Reserved: 0,
        ui32IEEE1588TimeLo: 0,
        ui32IEEE1588TimeHi: 0,
    }
}

const NUM_TX_DESCRIPTORS: usize = 10;
const NUM_RX_DESCRIPTORS: usize = 10;
#[no_mangle]
pub static mut RX_DESCRIPTORS: [tEMACDMADescriptor; NUM_TX_DESCRIPTORS] =
    [d(), d(), d(), d(), d(), d(), d(), d(), d(), d()];
#[no_mangle]
pub static mut TX_DESCRIPTORS: [tEMACDMADescriptor; NUM_RX_DESCRIPTORS] =
    [d(), d(), d(), d(), d(), d(), d(), d(), d(), d()];
#[no_mangle]
pub static mut RX_DESCRIPTOR_INDEX: usize = 0;
#[no_mangle]
pub static mut TX_DESCRIPTOR_INDEX: usize = 0;

const RX_BUFFER_SIZE: usize = 1536;
#[no_mangle]
pub static mut RX_BUFFERS: [[u8; RX_BUFFER_SIZE]; NUM_RX_DESCRIPTORS] =
    [[0; RX_BUFFER_SIZE]; NUM_RX_DESCRIPTORS];

interrupt!(EMAC0, emac_0);

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

    let mut serial = tm4c129x_hal::serial::Serial::uart0(
        p.UART0,
        uart_tx_pin,
        uart_rx_pin,
        (),
        (),
        115_200.bps(),
        tm4c129x_hal::serial::NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control,
    );

    let _ = writeln!(
        serial,
        "Chip information: {:#?}",
        tm4c129x_hal::sysctl::chip_id::get()
    );

    unsafe {
        SERIAL = Some(serial);
    }

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

    init_ethernet(&sc.power_control, clocks);

    let tx_buf: [u8; 42] = [
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x1A, 0xB6, 0x00, 0x02, 0x74, 0x08, 0x00, 0x45,
        0x00, 0x00, 0x1C, 0x00, 0x01, 0x00, 0x00, 0x40, 0x01, 0x25, 0xE0, 0x0A, 0x05, 0x02, 0x02,
        0x0A, 0x05, 0x02, 0x01, 0x08, 0x00, 0xF7, 0xFF, 0x00, 0x00, 0x00, 0x00,
    ];

    loop {
        for led in &mut leds {
            led.set_high();

            for _ in 0..5_000_000 {
                cortex_m::asm::nop();
            }
            led.set_low();
        }

        transmit_ethernet(&tx_buf);
    }
}
