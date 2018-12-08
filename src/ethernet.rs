use crate::println;
use byteorder::ByteOrder;
use smoltcp::phy::{ChecksumCapabilities, Device, DeviceCapabilities, RxToken, TxToken};
use smoltcp::time::Instant;
use tivaware_sys::*;
use tm4c129x::EMAC0;
use vcell::VolatileCell;

#[repr(C)]
union emac_des3 {
    link: *mut emac_descriptor,
    buffer_ext: *mut u8,
}

#[repr(C)]
struct emac_descriptor {
    ctrl_status: VolatileCell<u32>,
    buffer_len: u32,
    buffer: *mut u8,
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

extern "C" {
    fn SysCtlDelay(ui32Count: u32);
}

fn emac_reset(emac0: &EMAC0) {
    emac0.dmabusmod.modify(|_, w| w.swr().set_bit());

    while emac0.dmabusmod.read().swr().bit_is_set() {}
}

fn emac_phy_config_set(
    emac0: &EMAC0,
    lock: &tm4c129x_hal::sysctl::PowerControl,
    config: impl for<'r, 'w> FnOnce(
        &'r tm4c129x::emac0::pc::R,
        &'w mut tm4c129x::emac0::pc::W,
    ) -> &'w mut tm4c129x::emac0::pc::W,
) {
    emac0.pc.modify(config);

    let pc = emac0.pc.read();
    if pc.phyext().bit_is_clear() {
        tm4c129x_hal::sysctl::reset(lock, tm4c129x_hal::sysctl::Domain::Ephy0);
        unsafe { SysCtlDelay(10000) };
    }

    // TI's register definitions seem to disagree with the datasheet here - this
    // register should be RW, and also doesn't seem to have the CLKEN field we need.
    // For now just assert that the bit is already set to the value we expect.
    if pc.pintfs().is_rmii() {
        // emac0.cc.modify(|_, w| w.clken().set_bit());
        assert!(emac0.cc.read().bits() & 0x00010000 == 0x00010000);
    } else {
        // emac0.cc.modify(|_, w| w.clken().clear_bit());
        assert!(emac0.cc.read().bits() & 0x00010000 == 0);
    }

    tm4c129x_hal::sysctl::reset(lock, tm4c129x_hal::sysctl::Domain::Emac0);

    unsafe { SysCtlDelay(1000) };
}

fn emac_init(
    _emac: &EMAC0,
    _sysclk: u32,
    _bus_config: u32,
    rx_burst: u32,
    tx_burst: u32,
    desc_skip_size: u32,
) {
    // uint32_t ui32Val, ui32Div;

    // //
    // // Parameter sanity checks.
    // //
    // ASSERT(ui32DescSkipSize < 32);
    assert!(desc_skip_size < 32);
    // ASSERT(ui32TxBurst < (32 * 8));
    assert!(tx_burst < 32 * 8);
    // ASSERT(ui32RxBurst < (32 * 8));
    assert!(rx_burst < 32 * 8);

    // //
    // // Make sure that the DMA software reset is clear before continuing.
    // //
    // while(HWREG(EMAC0_BASE + EMAC_O_DMABUSMOD) & EMAC_DMABUSMOD_SWR)
    // {
    // }

    // //
    // // Set common flags.  Note that this driver assumes we are always using
    // // 8 word descriptors so we need to OR in EMAC_DMABUSMOD_ATDS here.
    // //
    // ui32Val = (ui32BusConfig | (ui32DescSkipSize << EMAC_DMABUSMOD_DSL_S) |
    //            EMAC_DMABUSMOD_ATDS);

    // //
    // // Do we need to use the 8X burst length multiplier?
    // //
    // if((ui32TxBurst > 32) || (ui32RxBurst > 32))
    // {
    //     //
    //     // Divide both burst lengths by 8 and set the 8X burst length
    //     // multiplier.
    //     //
    //     ui32Val |= EMAC_DMABUSMOD_8XPBL;
    //     ui32TxBurst >>= 3;
    //     ui32RxBurst >>= 3;

    //     //
    //     // Sanity check - neither burst length should have become zero.  If
    //     // they did, this indicates that the values passed are invalid.
    //     //
    //     ASSERT(ui32RxBurst);
    //     ASSERT(ui32TxBurst);
    // }

    // //
    // // Are the receive and transmit burst lengths the same?
    // //
    // if(ui32RxBurst == ui32TxBurst)
    // {
    //     //
    //     // Yes - set up to use a single burst length.
    //     //
    //     ui32Val |= (ui32TxBurst << EMAC_DMABUSMOD_PBL_S);
    // }
    // else
    // {
    //     //
    //     // No - we need to use separate burst lengths for each.
    //     //
    //     ui32Val |= (EMAC_DMABUSMOD_USP |
    //                 (ui32TxBurst << EMAC_DMABUSMOD_PBL_S) |
    //                 (ui32RxBurst << EMAC_DMABUSMOD_RPBL_S));
    // }

    // //
    // // Finally, write the bus mode register.
    // //
    // HWREG(ui32Base + EMAC_O_DMABUSMOD) = ui32Val;

    // //
    // // Default the MII CSR clock divider based on the fastest system clock.
    // //
    // ui32Div = g_pi16MIIClockDiv[NUM_CLOCK_DIVISORS - 1].ui32Divisor;

    // //
    // // Find the MII CSR clock divider to use based on the current system clock.
    // //
    // for(ui32Val = 0; ui32Val < NUM_CLOCK_DIVISORS; ui32Val++)
    // {
    //     if(ui32SysClk <= g_pi16MIIClockDiv[ui32Val].ui32SysClockMax)
    //     {
    //         ui32Div = g_pi16MIIClockDiv[ui32Val].ui32Divisor;
    //         break;
    //     }
    // }

    // //
    // // Set the MII CSR clock speed.
    // //
    // HWREG(ui32Base + EMAC_O_MIIADDR) = ((HWREG(ui32Base + EMAC_O_MIIADDR) &
    //                                      ~EMAC_MIIADDR_CR_M) | ui32Div);

    // //
    // // Disable all the MMC interrupts as these are enabled by default at reset.
    // //
    // HWREG(ui32Base + EMAC_O_MMCRXIM) = 0xFFFFFFFF;
    // HWREG(ui32Base + EMAC_O_MMCTXIM) = 0xFFFFFFFF;
}

fn emac_config_set(_emac0: &EMAC0, _config: u32, _mode_flags: u32, _rx_max_frame_size: u32) {
    //     //
    //     // Parameter sanity check.  Note that we allow TX_ENABLED and RX_ENABLED
    //     // here because we'll mask them off before writing the value and this
    //     // makes back-to-back EMACConfigGet/EMACConfigSet calls work without the
    //     // caller needing to explicitly remove these bits from the parameter.
    //     //
    //     ASSERT((ui32Config & ~(VALID_CONFIG_FLAGS |  EMAC_CONFIG_TX_ENABLED |
    //                            EMAC_CONFIG_RX_ENABLED)) == 0);
    //     ASSERT(!ui32RxMaxFrameSize || ((ui32RxMaxFrameSize < 0x4000) &&
    //                                    (ui32RxMaxFrameSize > 1522)));

    //     //
    //     // Set the configuration flags as specified.  Note that we
    // unconditionally     // OR in the EMAC_CFG_PS bit here since this
    // implementation supports only     // MII and RMII interfaces to the PHYs.
    //     //
    //     HWREG(ui32Base + EMAC_O_CFG) =
    //         ((HWREG(ui32Base + EMAC_O_CFG) & ~VALID_CONFIG_FLAGS) | ui32Config |
    //          EMAC_CFG_PS);

    //     //
    //     // Set the maximum receive frame size.  If 0 is passed, this implies
    //     // that the default maximum frame size should be used so just turn off
    //     // the override.
    //     //
    //     if(ui32RxMaxFrameSize)
    //     {
    //         HWREG(ui32Base + EMAC_O_WDOGTO) = ui32RxMaxFrameSize |
    // EMAC_WDOGTO_PWE;     }
    //     else
    //     {
    //         HWREG(ui32Base + EMAC_O_WDOGTO) &= ~EMAC_WDOGTO_PWE;
    //     }

    //     //
    //     // Set the operating mode register.
    //     //
    //     HWREG(ui32Base + EMAC_O_DMAOPMODE) = ui32ModeFlags;
}

fn emac_primary_addr_set(emac0: &EMAC0, mac_addr: [u8; 6]) {
    unsafe {
        emac0.addr0h.write(|w| {
            w.addrhi()
                .bits(byteorder::LittleEndian::read_u16(&mac_addr[4..]))
        });
        emac0.addr0l.write(|w| {
            w.addrlo()
                .bits(byteorder::LittleEndian::read_u32(&mac_addr[..4]))
        });
    }
}

fn emac_phy_read(emac0: &EMAC0, phy_addr: u8, reg_addr: u8) -> u16 {
    assert!(phy_addr < 32);
    assert!(reg_addr < 32);

    while emac0.miiaddr.read().miib().bit_is_set() {}

    unsafe {
        emac0.miiaddr.modify(|_, w| {
            w.pla().bits(phy_addr);
            w.mii().bits(reg_addr);

            w.miiw().clear_bit();
            w.miib().set_bit();

            w
        });
    }

    while emac0.miiaddr.read().miib().bit_is_set() {}

    emac0.miidata.read().data().bits()
}

fn emac_frame_filter_set(
    emac0: &EMAC0,
    filter_opts: impl for<'r, 'w> FnOnce(
        &'r tm4c129x::emac0::framefltr::R,
        &'w mut tm4c129x::emac0::framefltr::W,
    ) -> &'w mut tm4c129x::emac0::framefltr::W,
) {
    emac0.framefltr.modify(filter_opts)
}

fn emac_tx_enable(emac0: &EMAC0) {
    emac0.dmaopmode.modify(|_, w| w.st().set_bit());
    emac0.cfg.modify(|_, w| w.te().set_bit());
}

fn emac_rx_enable(emac0: &EMAC0) {
    emac0.dmaopmode.modify(|_, w| w.sr().set_bit());
    emac0.cfg.modify(|_, w| w.re().set_bit());
}

pub struct Tm4cEthernetDevice {
    tx_index: usize,
    rx_index: usize,
    emac0: EMAC0,
}

impl Tm4cEthernetDevice {
    pub fn new(
        lock: &tm4c129x_hal::sysctl::PowerControl,
        clocks: tm4c129x_hal::sysctl::Clocks,
        nvic: &mut cortex_m::peripheral::NVIC,
        emac0: EMAC0,
    ) -> Tm4cEthernetDevice {
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

        emac_reset(&emac0);

        emac_phy_config_set(&emac0, lock, |_, w| {
            // EMAC_PHY_TYPE_INTERNAL
            w.phyext().clear_bit();
            w.pintfs().imii();
            // EMAC_PHY_INT_MDIX_EN
            w.mdixen().set_bit();

            // EMAC_PHY_AN_100B_T_FULL_DUPLEX
            w.anen().set_bit();
            w.anmode()._100fd();

            w
        });

        emac_init(
            &emac0,
            clocks.sysclk.0,
            EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED,
            4,
            4,
            1,
        );

        emac_config_set(
            &emac0,
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

        unsafe {
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

            emac0
                .rxdladdr
                .write(|w| w.bits(&mut RX_DESCRIPTORS as *mut _ as u32));
            emac0
                .txdladdr
                .write(|w| w.bits(&mut TX_DESCRIPTORS as *mut _ as u32));
        }

        emac_primary_addr_set(&emac0, [0x00u8, 0x1A, 0xB6, 0x00, 0x02, 0x74]);

        while emac_phy_read(&emac0, 0, EPHY_BMSR as u8) & EPHY_BMSR_LINKSTAT as u16 == 0 {
            cortex_m::asm::nop();
        }

        emac_frame_filter_set(&emac0, |_, w| {
            w.ra().set_bit();
            w.pr().set_bit();

            w
        });

        // unsafe {
        //     EMACIntClear(emac0_base, EMACIntStatus(emac0_base, false));
        // }
        emac_tx_enable(&emac0);
        emac_rx_enable(&emac0);
        nvic.enable(tm4c129x::Interrupt::EMAC0);

        Tm4cEthernetDevice {
            rx_index: 0,
            tx_index: 0,
            emac0,
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
                emac0: &mut self.emac0,
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
            emac0: &mut self.emac0,
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

pub struct Tm4cRxToken<'a> {
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

pub struct Tm4cTxToken<'a> {
    descriptor: &'a mut emac_descriptor,
    emac0: &'a mut EMAC0,
}

impl<'a> TxToken for Tm4cTxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        println!("tx start consume");
        let result;

        assert!(len <= RX_BUFFER_SIZE);

        let mut data: [u8; RX_BUFFER_SIZE] = unsafe { core::mem::uninitialized() };
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
        self.emac0.txpolld.write(|w| w);

        println!("tx end consume");

        result
    }
}
