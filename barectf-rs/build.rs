#![deny(warnings, clippy::all)]

use std::env;
use std::path::PathBuf;

fn main() {
    compile_barectf();

    generate_bindings();
}

fn compile_barectf() {
    println!("cargo:rerun-if-changed=c/source/barectf.c");

    cc::Build::new()
        .file("c/source/barectf.c")
        .include("c/include")
        .flag("-Wno-unused-function")
        .compile("barectf");
    println!("cargo:rustc-link-lib=static=barectf");
}

fn generate_bindings() {
    println!("cargo:rerun-if-changed=bindgen/wrapper.h");

    let bindings = bindgen::Builder::default()
        .header("bindgen/wrapper.h")
        .clang_arg("-Ic/include")
        .allowlist_var("barectf.*")
        .allowlist_function("barectf.*")
        .allowlist_type("barectf.*")
        .ctypes_prefix("cty")
        .derive_copy(false)
        .use_core()
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate barectf bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write barectf bindings");
}
