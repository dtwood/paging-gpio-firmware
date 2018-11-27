#![no_std]
#![no_main]

extern crate panic_semihosting;

use core::fmt::Write;
use cortex_m_rt::entry;
use tm4c129x_hal::gpio;
use tm4c129x_hal::prelude::*;
use tm4c129x_hal::sysctl::SysctlExt;
use tm4c129x_hal::sysctl::{CrystalFrequency, Oscillator, PllOutputFrequency, SystemClock};

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
        115200.bps(),
        tm4c129x_hal::serial::NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control,
    );

    writeln!(serial, "Hello, {}!", "world").unwrap();

    let mut leds: [&mut embedded_hal::digital::OutputPin; 4] = [
        &mut portn.pn1.into_push_pull_output(),
        &mut portn.pn0.into_push_pull_output(),
        &mut portf.pf4.into_push_pull_output(),
        &mut portf.pf0.into_push_pull_output(),
    ];
    let button1 = portj.pj0.into_pull_up_input();
    let button2 = portj.pj1.into_pull_up_input();

    loop {
        let limit = 1 + if button1.is_low() { 1 } else { 0 } + if button2.is_low() { 2 } else { 0 };

        for led in &mut leds[0..limit] {
            led.set_high();
            for _ in 0..500000 {
                cortex_m::asm::nop();
            }
            led.set_low();
        }
    }
}
