#![no_std]
#![no_main]

extern crate panic_semihosting;

use core::fmt::Write;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
use tivaware_sys;
use tm4c129x;
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

    writeln!(
        serial,
        "Chip information: {:#?}",
        tm4c129x_hal::sysctl::chip_id::get()
    )
    .unwrap();

    let mut leds: [&mut embedded_hal::digital::OutputPin; 4] = [
        &mut portn.pn1.into_push_pull_output(),
        &mut portn.pn0.into_push_pull_output(),
        &mut portf.pf4.into_push_pull_output(),
        &mut portf.pf0.into_push_pull_output(),
    ];
    let button1 = portj.pj0.into_pull_up_input();
    let button2 = portj.pj1.into_pull_up_input();

    for led in &mut leds {
        led.set_high();
        for _ in 0..500_000 {
            cortex_m::asm::nop();
        }
        led.set_low();
    }

    let _ = hprintln!("{}", line!());

    unsafe {
        // Enable and reset the MAC.
        tivaware_sys::SysCtlPeripheralEnable(tivaware_sys::SYSCTL_PERIPH_EMAC0);
        tivaware_sys::SysCtlPeripheralReset(tivaware_sys::SYSCTL_PERIPH_EMAC0);

        // Enable and reset the internal PHY.
        tivaware_sys::SysCtlPeripheralEnable(tivaware_sys::SYSCTL_PERIPH_EPHY0);
        tivaware_sys::SysCtlPeripheralReset(tivaware_sys::SYSCTL_PERIPH_EPHY0);

        // Ensure the MAC is completed its reset.
        while !tivaware_sys::SysCtlPeripheralReady(tivaware_sys::SYSCTL_PERIPH_EMAC0) {}
        // Set the PHY type and configuration options.
        tivaware_sys::EMACPHYConfigSet(
            tivaware_sys::EMAC0_BASE,
            tivaware_sys::EMAC_PHY_TYPE_INTERNAL,
        );
        // Initialize and configure the MAC.
        tivaware_sys::EMACInit(tivaware_sys::EMAC0_BASE, clocks.sysclk.0, 0, 0, 0, 0);
        tivaware_sys::EMACConfigSet(
            tivaware_sys::EMAC0_BASE,
            tivaware_sys::EMAC_CONFIG_USE_MACADDR0
                | tivaware_sys::EMAC_CONFIG_IF_GAP_40BITS
                | tivaware_sys::EMAC_CONFIG_3BYTE_PREAMBLE
                | tivaware_sys::EMAC_CONFIG_BO_LIMIT_2
                | tivaware_sys::EMAC_CONFIG_SA_INSERT
                | tivaware_sys::EMAC_CONFIG_FULL_DUPLEX
                | tivaware_sys::EMAC_MODE_TX_THRESHOLD_16_BYTES
                | tivaware_sys::EMAC_MODE_RX_THRESHOLD_64_BYTES,
            0,
            0,
        );
    }

    let _ = hprintln!("{}", line!());

    loop {
        let limit = 1 + if button1.is_low() { 1 } else { 0 } + if button2.is_low() { 2 } else { 0 };

        for led in &mut leds[0..limit] {
            led.set_high();
            for _ in 0..500_000 {
                cortex_m::asm::nop();
            }
            led.set_low();
        }
    }
}
