fn main() {
    // The static SimConnect.lib from the MSFS SDK depends on these Windows
    // system libraries (e.g. GetRegistryPort -> advapi32). Depending on the
    // toolchain and SDK version they are not linked implicitly, which fails
    // with LNK2019 (__imp_RegCloseKey and friends) — link them explicitly.
    if std::env::var_os("CARGO_FEATURE_SIMCONNECT").is_some() {
        println!("cargo:rustc-link-lib=advapi32");
        println!("cargo:rustc-link-lib=user32");
        println!("cargo:rustc-link-lib=shell32");
        println!("cargo:rustc-link-lib=ws2_32");
        println!("cargo:rustc-link-lib=shlwapi");
    }
}
