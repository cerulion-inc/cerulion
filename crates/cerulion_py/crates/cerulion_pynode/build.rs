fn main() {
    println!("cargo:rustc-check-cfg=cfg(Py_LIMITED_API)");
    println!("cargo:rustc-check-cfg=cfg(cerulion_pynode_limited)");
    if pyo3_build_config::get()
        .target_abi()
        .to_string()
        .contains("-abi3-")
    {
        println!("cargo:rustc-cfg=cerulion_pynode_limited");
    }
    let config = pyo3_build_config::get();
    let name = config.lib_name().unwrap_or("python3");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        let fallback = format!("lib{name}.dylib");
        let primary = match config.lib_dir() {
            Some(dir) => format!("{dir}/{fallback}"),
            None => fallback.clone(),
        };
        println!("cargo:rustc-env=CERULION_LIBPYTHON_SONAME={primary}");
        println!("cargo:rustc-env=CERULION_LIBPYTHON_FALLBACK={fallback}");
    } else {
        println!("cargo:rustc-env=CERULION_LIBPYTHON_SONAME=lib{name}.so.1.0");
        println!("cargo:rustc-env=CERULION_LIBPYTHON_FALLBACK=lib{name}.so");
    }
}
