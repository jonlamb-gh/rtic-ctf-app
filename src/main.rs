#![no_main]
#![no_std]

//use panic_abort as _; // panic handler
use panic_rtt_target as _; // panic handler

mod serial;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use crate::serial::dma::TxTransfer;
    use crate::sys_clock::SystemClock;
    use barectf_rs::{BarectfPlatform, BarectfPlatformResources};
    use bbqueue::framed::FrameConsumer;
    use bbqueue::BBBuffer;
    use log::info;
    use rtt_logger::RTTLogger;
    use rtt_target::rtt_init_print;
    use stm32f4xx_hal::{
        dma::StreamsTuple,
        gpio::{Output, Pin, PushPull},
        pac::{self, TIM3, TIM4, TIM5},
        prelude::*,
        serial::config::{Config, StopBits},
        serial::{self, Serial},
        timer::counter::{CounterHz, CounterUs},
        timer::{Event, MonoTimerUs},
    };

    /// LED on PC13
    type LedPin = Pin<Output<PushPull>, 'C', 13>;

    const CTF_PACKET_SIZE: usize = 64;

    const DMA_TX_Q_LEN: usize = 4;
    const DMA_TX_Q_SIZE: usize = DMA_TX_Q_LEN * CTF_PACKET_SIZE;

    const LOGGER: RTTLogger = RTTLogger::new(log::LevelFilter::Trace);
    static SYS_CLOCK: SystemClock = SystemClock::new();

    #[shared]
    struct Shared {
        #[lock_free]
        tx: TxTransfer<DMA_TX_Q_SIZE>,
        #[lock_free]
        tx_cons: FrameConsumer<'static, DMA_TX_Q_SIZE>,
        #[lock_free]
        ctf_platform: BarectfPlatform<'static, 'static, DMA_TX_Q_SIZE, CTF_PACKET_SIZE>,
    }

    #[local]
    struct Local {
        led: LedPin,
        clock_timer: CounterUs<TIM3>,
        ctf_io_timer: CounterHz<TIM4>,
        worker_timer: CounterHz<TIM5>,
    }

    #[monotonic(binds = TIM2, default = true)]
    type MicrosecMono = MonoTimerUs<pac::TIM2>;

    fn get_clock() -> u32 {
        SYS_CLOCK.get_raw()
    }

    #[init(local = [
        tx_q: BBBuffer<DMA_TX_Q_SIZE> = BBBuffer::new(),
        ctf_platform_res: BarectfPlatformResources<'static, DMA_TX_Q_SIZE, CTF_PACKET_SIZE> =
            BarectfPlatformResources::new(get_clock),
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        log::set_logger(&LOGGER)
            .map(|()| log::set_max_level(log::LevelFilter::Trace))
            .unwrap();

        info!("Starting");

        // Set up the system clock
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(25.MHz())
            .sysclk(48.MHz())
            .require_pll48clk()
            .freeze();

        let (tx_prod, tx_cons) = ctx.local.tx_q.try_split_framed().unwrap();

        let gpioa = ctx.device.GPIOA.split();
        //let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();

        let mut led = gpioc.pc13.into_push_pull_output();
        led.set_low();

        // USART2, 115200, 8N1, Rx/Tx DMA
        let tx = gpioa.pa2.into_alternate();
        let rx = gpioa.pa3.into_alternate();
        let mut serial_config = Config::default()
            .baudrate(115_200.bps())
            .wordlength_8()
            .parity_none()
            .stopbits(StopBits::STOP1);
        serial_config.dma = serial::config::DmaConfig::TxRx;
        let serial = Serial::new(ctx.device.USART2, (tx, rx), serial_config, &clocks)
            .unwrap()
            .with_u8_data();
        let (serial_tx, mut serial_rx) = serial.split();
        serial_rx.listen_idle();

        let dma_streams = StreamsTuple::new(ctx.device.DMA1);

        // Setup Tx DMA, gets initialized when started
        let tx = TxTransfer::Uninitialized(dma_streams.6, serial_tx);

        let ctf_platform = ctx.local.ctf_platform_res.init(tx_prod);

        let mut clock_timer = ctx.device.TIM3.counter_us(&clocks);
        clock_timer.start(1.millis()).unwrap();
        clock_timer.listen(Event::Update);

        let mut ctf_io_timer = ctx.device.TIM4.counter_hz(&clocks);
        ctf_io_timer.start(10.Hz()).unwrap();
        ctf_io_timer.listen(Event::Update);

        let mut worker_timer = ctx.device.TIM5.counter_hz(&clocks);
        worker_timer.start(4.Hz()).unwrap();
        worker_timer.listen(Event::Update);

        let mono = ctx.device.TIM2.monotonic_us(&clocks);
        info!("Initialized");

        (
            Shared {
                tx,
                tx_cons,
                ctf_platform,
            },
            Local {
                led,
                clock_timer,
                ctf_io_timer,
                worker_timer,
            },
            init::Monotonics(mono),
        )
    }

    use crate::serial::start_dma_tx;
    extern "Rust" {
        #[task(local = [led], shared = [tx, tx_cons], capacity = 2, priority = 3)]
        fn start_dma_tx(ctx: start_dma_tx::Context);
    }

    use crate::serial::on_dma_tx_complete;
    extern "Rust" {
        #[task(binds=DMA1_STREAM6, shared = [tx], priority = 3)]
        fn on_dma_tx_complete(ctx: on_dma_tx_complete::Context);
    }

    #[task(binds=TIM5, local = [worker_timer, n: u32 = 0], shared = [ctf_platform], priority = 2)]
    fn on_worker_timer(ctx: on_worker_timer::Context) {
        info!("Doing work");
        let n = ctx.local.n;
        let timer = ctx.local.worker_timer;
        let tracer = ctx.shared.ctf_platform.ctx_mut();
        let _ = timer.wait();
        tracer.trace_event_a();
        tracer.trace_event_b(*n);
        *n = n.saturating_add(1);
    }

    #[task(binds=TIM3, local = [clock_timer], priority = 4)]
    fn on_clock_timer(ctx: on_clock_timer::Context) {
        let timer = ctx.local.clock_timer;
        let _ = timer.wait();
        SYS_CLOCK.inc_from_interrupt();
    }

    #[task(binds=TIM4, local = [ctf_io_timer], priority = 2)]
    fn on_ctf_io_timer(ctx: on_ctf_io_timer::Context) {
        let timer = ctx.local.ctf_io_timer;
        let _ = timer.wait();
        start_dma_tx::spawn().ok();
    }
}

mod sys_clock {
    use core::sync::atomic::{AtomicU32, Ordering::SeqCst};

    /// 32-bit millisecond clock
    #[derive(Debug)]
    #[repr(transparent)]
    pub struct SystemClock(AtomicU32);

    impl SystemClock {
        pub const fn new() -> Self {
            SystemClock(AtomicU32::new(0))
        }

        pub fn inc_from_interrupt(&self) {
            self.0.fetch_add(1, SeqCst);
        }

        pub fn get_raw(&self) -> u32 {
            self.0.load(SeqCst)
        }
    }
}

// Could also define NDEBUG in barectf-rs
#[no_mangle]
extern "C" fn __assert_func(_file: *const i8, _line: isize, _e: *const i8) {
    panic!("assert called");
}
