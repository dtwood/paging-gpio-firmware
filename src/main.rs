#![no_std]
#![no_main]

extern crate panic_halt;

use cortex_m;
use cortex_m_rt::entry;
use tm4c129x;

#[entry]
fn main() -> ! {
    let p = tm4c129x::Peripherals::take().unwrap();

    p.SYSCTL
        .rcgcgpio
        .write(|w| w.r5().set_bit().r12().set_bit());
    p.GPIO_PORTF_AHB
        .den
        .write(|w| w.den().bits(1 << 0 | 1 << 4));
    p.GPIO_PORTF_AHB
        .dir
        .write(|w| w.dir().bits(1 << 0 | 1 << 4));
    p.GPIO_PORTN.den.write(|w| w.den().bits(1 << 0 | 1 << 1));
    p.GPIO_PORTN.dir.write(|w| w.dir().bits(1 << 0 | 1 << 1));

    loop {
        p.GPIO_PORTN.data.write(|w| w.data().bits(1 << 1));
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }
        p.GPIO_PORTN.data.write(|w| w.data().bits(1 << 0));
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }
        p.GPIO_PORTN.data.write(|w| w.data().bits(0));

        p.GPIO_PORTF_AHB.data.write(|w| w.data().bits(1 << 4));
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }
        p.GPIO_PORTF_AHB.data.write(|w| w.data().bits(1 << 0));
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }
        p.GPIO_PORTF_AHB.data.write(|w| w.data().bits(0));
    }
}
