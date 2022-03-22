#![no_std]
#![deny(warnings, clippy::all)]

use bbqueue::framed::{FrameGrantW, FrameProducer};
use core::{cell::UnsafeCell, mem::MaybeUninit, ptr, slice};

pub mod ffi;

pub struct BarectfPlatformResources<'buf, const N: usize, const P: usize> {
    ctx: UnsafeCell<MaybeUninit<BarectfCtx>>,
    prod: UnsafeCell<MaybeUninit<FrameProducer<'buf, N>>>,
    get_clock_fn: fn() -> u32,
    pkt_grant: UnsafeCell<MaybeUninit<Option<FrameGrantW<'buf, N>>>>,
}

unsafe impl<'buf, const N: usize, const P: usize> Send for BarectfPlatformResources<'buf, N, P> {}
unsafe impl<'buf, const N: usize, const P: usize> Sync for BarectfPlatformResources<'buf, N, P> {}

pub struct BarectfPlatform<'a, 'buf, const N: usize, const P: usize> {
    res: &'a BarectfPlatformResources<'buf, N, P>,
}

impl<'buf, const N: usize, const P: usize> BarectfPlatformResources<'buf, N, P> {
    pub const fn new(get_clock_fn: fn() -> u32) -> Self {
        Self {
            ctx: UnsafeCell::new(MaybeUninit::uninit()),
            get_clock_fn,
            prod: UnsafeCell::new(MaybeUninit::uninit()),
            pkt_grant: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    /// # Safety: only call this once
    pub fn init<'a>(&'a self, prod: FrameProducer<'buf, N>) -> BarectfPlatform<'a, 'buf, N, P> {
        unsafe {
            let p = self.prod.get();
            (&mut *p).write(prod);

            let c = self.ctx.get();
            (&mut *c).write(BarectfCtx::zero());

            let g = self.pkt_grant.get();
            (&mut *g).write(None);
        }

        let mut p = BarectfPlatform { res: self };

        let cbs = ffi::barectf_platform_callbacks {
            default_clock_get_value: Some(platform_callbacks::get_default_clock::<N, P>),
            is_backend_full: Some(platform_callbacks::is_backend_full::<N, P>),
            open_packet: Some(platform_callbacks::open_packet::<N, P>),
            close_packet: Some(platform_callbacks::close_packet::<N, P>),
        };

        let res_ptr_for_callbacks = self as *const _ as *mut _;
        let ctx_ptr = p.ctx_mut().as_mut_vptr();

        // Initial buffer provided in the following open_packet() call
        unsafe {
            ffi::barectf_init(
                ctx_ptr,
                ptr::null_mut(),
                P as u32,
                cbs,
                res_ptr_for_callbacks,
            )
        };

        platform_callbacks::open_packet::<N, P>(res_ptr_for_callbacks);

        p
    }

    /// Safety: ptr must point to BarectfPlatformResources
    fn from_raw<'a>(ptr: *mut cty::c_void) -> &'a BarectfPlatformResources<'buf, N, P> {
        unsafe { &*(ptr as *const BarectfPlatformResources<'buf, N, P>) }
    }

    #[allow(clippy::mut_from_ref)]
    fn producer(&self) -> &mut FrameProducer<'buf, N> {
        unsafe {
            let p = self.prod.get().cast::<FrameProducer<'buf, N>>();
            &mut *p
        }
    }

    #[allow(clippy::mut_from_ref)]
    fn pkt_grant(&self) -> &mut Option<FrameGrantW<'buf, N>> {
        unsafe {
            let p = self.pkt_grant.get().cast::<Option<FrameGrantW<'buf, N>>>();
            &mut *p
        }
    }

    #[allow(clippy::mut_from_ref)]
    fn ctx_mut(&self) -> &mut BarectfCtx {
        unsafe {
            let c = self.ctx.get().cast::<BarectfCtx>();
            &mut *c
        }
    }
}

impl<'a, 'buf, const N: usize, const P: usize> BarectfPlatform<'a, 'buf, N, P> {
    pub fn ctx_mut(&mut self) -> &mut BarectfCtx {
        self.res.ctx_mut()
    }
}

pub(crate) mod platform_callbacks {
    use crate::{ffi, BarectfPlatformResources};
    use core::ops::DerefMut;

    pub extern "C" fn get_default_clock<const N: usize, const P: usize>(
        plat_res_ptr: *mut cty::c_void,
    ) -> u32 {
        debug_assert!(!plat_res_ptr.is_null());
        let p = BarectfPlatformResources::<N, P>::from_raw(plat_res_ptr);
        (p.get_clock_fn)()
    }

    pub extern "C" fn is_backend_full<const N: usize, const P: usize>(
        plat_res_ptr: *mut cty::c_void,
    ) -> cty::c_int {
        debug_assert!(!plat_res_ptr.is_null());
        let p = BarectfPlatformResources::<N, P>::from_raw(plat_res_ptr);
        p.producer().grant(P).is_err() as _
    }

    pub extern "C" fn open_packet<const N: usize, const P: usize>(plat_res_ptr: *mut cty::c_void) {
        debug_assert!(!plat_res_ptr.is_null());
        let p = BarectfPlatformResources::<N, P>::from_raw(plat_res_ptr);
        // Safety: we just checked that this will succeed in is_backend_full
        let mut g = p.producer().grant(P).unwrap();
        let buf = g.deref_mut();
        unsafe {
            ffi::barectf_packet_set_buf(p.ctx_mut().as_mut_vptr(), buf.as_mut_ptr(), P as _);
            ffi::barectf_default_open_packet(p.ctx_mut().as_mut_ptr());
        }
        let _ = p.pkt_grant().replace(g);
    }

    pub extern "C" fn close_packet<const N: usize, const P: usize>(plat_res_ptr: *mut cty::c_void) {
        debug_assert!(!plat_res_ptr.is_null());
        let p = BarectfPlatformResources::<N, P>::from_raw(plat_res_ptr);
        unsafe { ffi::barectf_default_close_packet(p.ctx_mut().as_mut_ptr()) };
        let g = p.pkt_grant().take().unwrap();
        g.commit(P);
    }
}

#[repr(transparent)]
#[derive(Debug)]
pub struct BarectfCtx(ffi::barectf_default_ctx);

impl BarectfCtx {
    const fn zero() -> Self {
        Self(ffi::barectf_default_ctx {
            parent: ffi::barectf_ctx {
                cbs: ffi::barectf_platform_callbacks {
                    default_clock_get_value: None,
                    is_backend_full: None,
                    open_packet: None,
                    close_packet: None,
                },
                data: ptr::null_mut(),
                buf: ptr::null_mut(),
                packet_size: 0,
                content_size: 0,
                at: 0,
                off_content: 0,
                events_discarded: 0,
                sequence_number: 0,
                packet_is_open: 0,
                in_tracing_section: 0,
                is_tracing_enabled: 0,
                use_cur_last_event_ts: 0,
            },
            off_ph_magic: 0,
            off_ph_stream_id: 0,
            off_pc_packet_size: 0,
            off_pc_content_size: 0,
            off_pc_timestamp_begin: 0,
            off_pc_timestamp_end: 0,
            off_pc_events_discarded: 0,
            off_pc_packet_seq_num: 0,
            cur_last_event_ts: 0,
        })
    }

    pub fn packet_size(&self) -> usize {
        unsafe { ffi::barectf_packet_size(self.as_vptr()) as usize }
    }

    pub fn packet_is_full(&self) -> bool {
        unsafe { ffi::barectf_packet_is_full(self.as_vptr()) != 0 }
    }

    pub fn packet_is_empty(&self) -> bool {
        unsafe { ffi::barectf_packet_is_empty(self.as_vptr()) != 0 }
    }

    pub fn packet_is_open(&self) -> bool {
        unsafe { ffi::barectf_packet_is_open(self.as_vptr()) != 0 }
    }

    pub fn packet_events_discarded(&self) -> usize {
        unsafe { ffi::barectf_packet_events_discarded(self.as_vptr()) as usize }
    }

    pub fn packet_sequence_number(&self) -> usize {
        unsafe { ffi::barectf_packet_sequence_number(self.as_vptr()) as usize }
    }

    pub fn packet_buf_size(&self) -> usize {
        unsafe { ffi::barectf_packet_buf_size(self.as_vptr()) as usize }
    }

    pub fn packet_buf(&self) -> &[u8] {
        let size = self.packet_buf_size();
        let ptr = unsafe { ffi::barectf_packet_buf(self.as_vptr()) };
        if size != 0 && !ptr.is_null() {
            unsafe { slice::from_raw_parts(ptr, size) }
        } else {
            &[]
        }
    }

    pub fn discarded_event_records_count(&self) -> usize {
        unsafe { ffi::barectf_discarded_event_records_count(self.as_vptr()) as usize }
    }

    pub fn packet_is_in_tracing_section(&self) -> bool {
        unsafe { ffi::barectf_is_in_tracing_section(self.as_vptr()) != 0 }
    }

    pub fn packet_is_tracing_enabled(&self) -> bool {
        unsafe { ffi::barectf_is_tracing_enabled(self.as_vptr()) != 0 }
    }

    pub fn enable_tracing(&mut self, enable: bool) {
        unsafe { ffi::barectf_enable_tracing(self.as_mut_vptr(), enable as _) }
    }

    fn as_ptr(&self) -> *const ffi::barectf_default_ctx {
        &self.0 as *const ffi::barectf_default_ctx
    }

    fn as_vptr(&self) -> *const cty::c_void {
        self.as_ptr() as *const cty::c_void
    }

    fn as_mut_ptr(&mut self) -> *mut ffi::barectf_default_ctx {
        &mut self.0 as *mut ffi::barectf_default_ctx
    }

    fn as_mut_vptr(&mut self) -> *mut cty::c_void {
        self.as_mut_ptr() as *mut cty::c_void
    }
}

// TDOO - generate these methods
impl BarectfCtx {
    pub fn trace_event_a(&mut self) {
        unsafe { ffi::barectf_default_trace_event_a(self.as_mut_ptr()) };
    }

    pub fn trace_event_b(&mut self, ms: u32) {
        unsafe { ffi::barectf_default_trace_event_b(self.as_mut_ptr(), ms) };
    }
}
