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
        // A framework build (python.org, Homebrew, Xcode) ships the interpreter
        // as `<Name>.framework/Versions/<ver>/<Name>` and usually also a
        // `lib/lib<name>.dylib` link beside it, but only the former is
        // guaranteed, so the host tries it when the links are absent.
        let framework = match (config.python_framework_prefix(), config.lib_dir()) {
            (Some(_), Some(dir)) => framework_binary(dir).unwrap_or_default(),
            _ => String::new(),
        };
        println!("cargo:rustc-env=CERULION_LIBPYTHON_SONAME={primary}");
        println!("cargo:rustc-env=CERULION_LIBPYTHON_FRAMEWORK={framework}");
        println!("cargo:rustc-env=CERULION_LIBPYTHON_FALLBACK={fallback}");
    } else {
        println!("cargo:rustc-env=CERULION_LIBPYTHON_SONAME=lib{name}.so.1.0");
        println!("cargo:rustc-env=CERULION_LIBPYTHON_FRAMEWORK=");
        println!("cargo:rustc-env=CERULION_LIBPYTHON_FALLBACK=lib{name}.so");
    }
}

/// The framework's interpreter binary for a framework `LIBDIR`
/// (`<prefix>/<Name>.framework/Versions/<ver>/lib`): the `<Name>` file in the
/// version directory, which is what `sysconfig`'s `LDLIBRARY` names. `None`
/// when `lib_dir` is not laid out that way.
fn framework_binary(lib_dir: &str) -> Option<String> {
    let lib_dir = std::path::Path::new(lib_dir);
    let version_dir = lib_dir.parent()?;
    let framework_dir = version_dir.parent()?.parent()?;
    let framework_name = framework_dir.file_name()?.to_str()?;
    let name = framework_name.strip_suffix(".framework")?;
    Some(version_dir.join(name).to_str()?.to_string())
}
