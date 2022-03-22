pub use c::*;

#[allow(non_upper_case_globals)]
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(deref_nullptr)]
mod c {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

#[cfg(target_pointer_width = "32")]
mod target_sanity {
    use static_assertions::const_assert_eq;

    const_assert_eq!(core::mem::size_of::<usize>(), core::mem::size_of::<u32>());
    const_assert_eq!(
        core::mem::size_of::<usize>(),
        core::mem::size_of::<cty::c_int>()
    );
}
