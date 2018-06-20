//! ADC driver for the nRF52. Uses the SAADC peripheral.

use kernel::common::StaticRef;
use kernel::hil;
use kernel::common::regs::{self, ReadOnly, ReadWrite, WriteOnly};
use kernel::common::cells::OptionalCell;

// https://github.com/andenore/NordicSnippets/blob/master/examples/saadc/main.c

#[repr(C)]
struct AdcRegisters {
    /// Start the ADC and prepare the result buffer in RAM
    tasks_start: WriteOnly<u32, TASK::Register>,
    /// Take one ADC sample, if scan is enabled all channels are sampled
    tasks_sample: WriteOnly<u32, TASK::Register>,
    /// Stop the ADC and terminate any on-going conversion
    tasks_stop: WriteOnly<u32, TASK::Register>,
    /// Starts offset auto-calibration
    tasks_calibrateoffset: WriteOnly<u32, TASK::Register>,
    _reserved0: [u8; 240],
    /// The ADC has started
    events_started: ReadWrite<u32, EVENT::Register>,
    /// The ADC has filled up the Result buffer
    events_end: ReadWrite<u32, EVENT::Register>,
    /// A conversion task has been completed. Depending on the mode, multiple conversion
    events_done: ReadWrite<u32, EVENT::Register>,
    /// A result is ready to get transferred to RAM
    events_resultdone: ReadWrite<u32, EVENT::Register>,
    /// Calibration is complete
    events_calibratedone: ReadWrite<u32, EVENT::Register>,
    /// The ADC has stopped
    events_stopped: ReadWrite<u32, EVENT::Register>,
    /// Last result is equal or above CH[X].LIMIT
    events_ch: [AdcEventChRegisters; 8],
    _reserved1: [u8; 424],
    /// Enable or disable interrupt
    inten: ReadWrite<u32, INTEN::Register>,
    /// Enable interrupt
    intenset: ReadWrite<u32, INTEN::Register>,
    /// Disable interrupt
    intenclr: ReadWrite<u32, INTEN::Register>,
    _reserved2: [u8; 244],
    /// Status
    status: ReadOnly<u32>,
    _reserved3: [u8; 252],
    /// Enable or disable ADC
    enable: ReadWrite<u32>,
    _reserved4: [u8; 12],
    ch: [AdcChRegisters; 8],
    _reserved4: [u8; 96],
    /// Resolution configuration
    resolution: ReadWrite<u32, RESOLUTION::Register>,
    /// Oversampling configuration. OVERSAMPLE should not be combined with SCAN. The RES
    oversample: ReadWrite<u32>,
    /// Controls normal or continuous sample rate
    samplerate: ReadWrite<u32, SAMPLERATE::Register>,
}

#[repr(C)]
struct AdcEventChRegisters {
    limith: ReadWrite<u32, EVENT::Register>,
    limitl: ReadWrite<u32, EVENT::Register>,
}

#[repr(C)]
struct AdcChRegisters {
    pselp: ReadWrite<u32, PSELP::Register>,
    pseln: ReadWrite<u32, PSELN::Register>,
    config: ReadWrite<u32, CONFIG::Register>,
    limit: ReadWrite<u32, LIMIT::Register>,
}

register_bitfields![u32,
    INTEN [
        /// Enable or disable interrupt on EVENTS_STARTED event
        STARTED 0,
        /// Enable or disable interrupt on EVENTS_END event
        END 1,
        /// Enable or disable interrupt on EVENTS_DONE event
        DONE 2,
        /// Enable or disable interrupt on EVENTS_RESULTDONE event
        RESULTDONE 3,
        /// Enable or disable interrupt on EVENTS_CALIBRATEDONE event
        CALIBRATEDONE 4,
        /// Enable or disable interrupt on EVENTS_STOPPED event
        STOPPED 5,
        /// Enable or disable interrupt on EVENTS_CH[0].LIMITH event
        CH0LIMITH 6,
        /// Enable or disable interrupt on EVENTS_CH[0].LIMITL event
        CH0LIMITL 7,
        /// Enable or disable interrupt on EVENTS_CH[1].LIMITH event
        CH1LIMITH 8,
        /// Enable or disable interrupt on EVENTS_CH[1].LIMITL event
        CH1LIMITL 9,
        /// Enable or disable interrupt on EVENTS_CH[2].LIMITH event
        CH2LIMITH 10,
        /// Enable or disable interrupt on EVENTS_CH[2].LIMITL event
        CH2LIMITL 11,
        /// Enable or disable interrupt on EVENTS_CH[3].LIMITH event
        CH3LIMITH 12,
        /// Enable or disable interrupt on EVENTS_CH[3].LIMITL event
        CH3LIMITL 13,
        /// Enable or disable interrupt on EVENTS_CH[4].LIMITH event
        CH4LIMITH 14,
        /// Enable or disable interrupt on EVENTS_CH[4].LIMITL event
        CH4LIMITL 15,
        /// Enable or disable interrupt on EVENTS_CH[5].LIMITH event
        CH5LIMITH 16,
        /// Enable or disable interrupt on EVENTS_CH[5].LIMITL event
        CH5LIMITL 17,
        /// Enable or disable interrupt on EVENTS_CH[6].LIMITH event
        CH6LIMITH 18,
        /// Enable or disable interrupt on EVENTS_CH[6].LIMITL event
        CH6LIMITL 19,
        /// Enable or disable interrupt on EVENTS_CH[7].LIMITH event
        CH7LIMITH 20,
        /// Enable or disable interrupt on EVENTS_CH[7].LIMITL event
        CH7LIMITL 21
    ],
    SAMPLERATE [
        /// Capture and compare value. Sample rate is 16 MHz/CC
        CC OFFSET(0) NUMBITS(11) [],
        /// Select mode for sample rate control
        MODE OFFSET(12) NUMBITS(1) [
            /// Rate is controlled from SAMPLE task
            RateIsControlledFromSAMPLETask = 0,
            /// Rate is controlled from local timer (use CC to control the rate)
            RateIsControlledFromLocalTimerUseCCToControlTheRate = 1
        ]
    ],
    EVENT [
        EVENT 0
    ],
    TASK [
        TASK 0
    ],
    PSEL [
        PSEL OFFSET(0) NUMBITS(5) [
            NotConnected = 0,
            AnalogInput0 = 1,
            AnalogInput1 = 2,
            AnalogInput2 = 3,
            AnalogInput3 = 4,
            AnalogInput4 = 5,
            AnalogInput5 = 6,
            AnalogInput6 = 7,
            AnalogInput7 = 8,
            VDD = 9,
            VDDHDIV5 = 0xD
        ]
    ],
    CONFIG [
        RESP OFFSET(0) NUMBITS(2) [
            Bypass = 0,
            Pulldown = 1,
            Pullup = 2,
            VDD1_2 = 3
        ],
        RESN OFFSET(4) NUMBITS(2) [
            Bypass = 0,
            Pulldown = 1,
            Pullup = 2,
            VDD1_2 = 3
        ],
        GAIN OFFSET(8) NUMBITS(3) [
            Gain1_6 = 0,
            Gain1_5 = 1,
            Gain1_4 = 2,
            Gain1_3 = 3,
            Gain1_2 = 4,
            Gain1 = 5,
            Gain2 = 6,
            Gain4 = 7
        ],
        REFSEL OFFSET(12) NUMBITS(1) [
            Internal = 0,
            VDD1_4 = 1
        ],
        TACQ OFFSET(16) NUMBITS(3) [
            us3 = 0,
            us5 = 1,
            us10 = 2,
            us15 = 3,
            us20 = 4,
            us40 = 5
        ],
        MODE OFFSET(20) NUMBITS(1) [
            SE = 0,
            Diff = 1
        ],
        BURST OFFSET(24) NUMBITS(1) [
            Disable = 0,
            Enable = 1
        ]
    ],
    LIMIT [
        LOW OFFSET(0) NUMBITS(16) [],
        HIGH OFFSET(16) NUMBITS(16) []
    ],
    RESOLUTION [
        VAL OFFSET(0) NUMBITS(3) [
            bit8 = 0,
            bit10 = 1,
            bit12 = 2,
            bit14 = 3,
        ]
    ]
];

#[derive(Copy, Clone, Debug)]
enum AdcChannel {
    AnalogInput0 = 1,
    AnalogInput1 = 2,
    AnalogInput2 = 3,
    AnalogInput3 = 4,
    AnalogInput4 = 5,
    AnalogInput5 = 6,
    AnalogInput6 = 7,
    AnalogInput7 = 8,
    VDD = 9,
    VDDHDIV5 = 0xD,
}

const SAADC_BASE: StaticRef<AdcRegisters> =
    unsafe { StaticRef::new(0x40007000 as *const AdcRegisters) };

pub struct Adc {
    registers: StaticRef<AdcRegisters>,

    // state tracking for the ADC
    enabled: Cell<bool>,
    adc_clk_freq: Cell<u32>,
    active: Cell<bool>,
    continuous: Cell<bool>,
    dma_running: Cell<bool>,
    cpu_clock: Cell<bool>,

    // timer fire counting for slow sampling rates
    timer_repeats: Cell<u8>,
    timer_counts: Cell<u8>,

    // DMA peripheral, buffers, and length
    rx_dma: Cell<Option<&'static dma::DMAChannel>>,
    rx_dma_peripheral: dma::DMAPeripheral,
    rx_length: Cell<usize>,
    next_dma_buffer: TakeCell<'static, [u16]>,
    next_dma_length: Cell<usize>,
    stopped_buffer: TakeCell<'static, [u16]>,

    // ADC client to send sample complete notifications to
    client: OptionalCell<&'static hil::adc::Client>,
}

impl Adc {
    const fn new(registers: StaticRef<AdcRegisters>) -> Adc {
        Adc {
            registers: registers,
        }
    }

    pub fn handle_interrupt(&self) {
    }
}

/// Implements an ADC capable reading ADC samples on any channel.
impl hil::adc::Adc for Adc {
    type Channel = AdcChannel;

    fn initialize(&self) -> ReturnCode {
        ReturnCode::SUCCESS
    }

    fn sample(&self, channel: &Self::Channel) -> ReturnCode {
        let regs: = &*self.registers;


        regs.ch[0].pselp.write(PSEL::PSEL.val(channel as u32));
        regs.ch[0].config.write(CONFIG::GAIN::Gain1_4 + CONFIG::REFSEL::VDD1_4
            + CONFIG::TACQ::us10);

        regs.resolution.write(RESOLUTION::VAL::bit14);





        // always configure to 1KHz to get the slowest clock with single sampling
        let res = self.config_and_enable(1000);

        if res != ReturnCode::SUCCESS {
            return res;
        } else if !self.enabled.get() {
            ReturnCode::EOFF
        } else if self.active.get() {
            // only one operation at a time
            ReturnCode::EBUSY
        } else {
            self.active.set(true);
            self.continuous.set(false);
            self.timer_repeats.set(0);
            self.timer_counts.set(0);

            let cfg = SequencerConfig::MUXNEG.val(0x7) + // ground pad
                SequencerConfig::MUXPOS.val(channel.chan_num)
                + SequencerConfig::INTERNAL.val(0x2 | channel.internal)
                + SequencerConfig::RES::Bits12
                + SequencerConfig::TRGSEL::Software
                + SequencerConfig::GCOMP::Disable
                + SequencerConfig::GAIN::Gain0p5x
                + SequencerConfig::BIPOLAR::Disable
                + SequencerConfig::HWLA::Disable;
            regs.seqcfg.write(cfg);

            // clear any current status
            self.clear_status();

            // enable end of conversion interrupt
            regs.ier.write(Interrupt::SEOC::SET);

            // initiate conversion
            regs.cr.write(Control::STRIG::SET);

            ReturnCode::SUCCESS
        }
    }

    fn sample_continuous(&self, channel: &Self::Channel, frequency: u32) -> ReturnCode {
        ReturnCode::FAIL
    }

    fn stop_sampling(&self) -> ReturnCode {
        ReturnCode::FAIL
    }
}
