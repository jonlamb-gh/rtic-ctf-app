#![no_std]
#![deny(warnings, clippy::all)]

use barectf_rs::*;
use bbqueue::framed::FrameConsumer;
use bbqueue::BBBuffer;
use core::sync::atomic::{AtomicU32, Ordering::SeqCst};

#[derive(Debug)]
#[repr(transparent)]
pub struct CounterClock(AtomicU32);

impl CounterClock {
    pub const fn new() -> Self {
        CounterClock(AtomicU32::new(0))
    }

    pub fn inc(&self) {
        self.0.fetch_add(1, SeqCst);
    }

    pub fn get(&self) -> u32 {
        self.0.load(SeqCst)
    }
}

const PKT_SIZE: usize = 64;
const Q_LEN: usize = 8;
const Q_SIZE: usize = PKT_SIZE * Q_LEN;

static CLOCK: CounterClock = CounterClock::new();

fn get_clock() -> u32 {
    let t = CLOCK.get();
    CLOCK.inc();
    t
}

static BB: BBBuffer<Q_SIZE> = BBBuffer::new();
static BCTF_PLATFORM: BarectfPlatformResources<'static, Q_SIZE, PKT_SIZE> =
    BarectfPlatformResources::new(get_clock);

fn get_resources() -> (
    FrameConsumer<'static, Q_SIZE>,
    BarectfPlatform<'static, 'static, Q_SIZE, PKT_SIZE>,
) {
    let (prod, cons) = BB.try_split_framed().unwrap();
    let platform = BCTF_PLATFORM.init(prod);
    (cons, platform)
}

#[test]
fn basic_test() {
    let (mut cons, mut platform) = get_resources();
    let ctx = platform.ctx_mut();

    assert_eq!(ctx.packet_sequence_number(), 0);
    ctx.trace_event_a();
    ctx.trace_event_a();
    ctx.trace_event_b(1);
    ctx.trace_event_b(2);
    ctx.trace_event_b(3);
    assert_eq!(ctx.packet_sequence_number(), 1);

    let g = cons.read().unwrap();
    assert_eq!(g.len(), PKT_SIZE);
    g.release();

    ctx.trace_event_a();
    ctx.trace_event_a();
    ctx.trace_event_b(1);
    ctx.trace_event_b(2);
    ctx.trace_event_b(3);
    assert_eq!(ctx.packet_sequence_number(), 2);

    let g = cons.read().unwrap();
    assert_eq!(g.len(), PKT_SIZE);
    g.release();
}
