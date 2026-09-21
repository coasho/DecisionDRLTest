// Links fsim.dll's import library from the CMake build tree (or FSIM_LIB_DIR)
// and copies the DLLs next to the executable so `cargo run` works as is.
use std::{env, fs, path::PathBuf};

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let build = env::var("FSIM_BUILD_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| manifest.join("../../build/ucrt64-release"));
    let lib = env::var("FSIM_LIB_DIR").map(PathBuf::from).unwrap_or_else(|_| build.join("lib"));
    let bin = env::var("FSIM_BIN_DIR").map(PathBuf::from).unwrap_or_else(|_| build.join("bin"));
    println!("cargo:rustc-link-search=native={}", lib.display());
    println!("cargo:rustc-link-lib=dylib=fsim");
    let vision = env::var("CARGO_FEATURE_VISION").is_ok();
    if vision {
        println!("cargo:rustc-link-lib=dylib=fsim_vision");
    }
    println!("cargo:rerun-if-env-changed=FSIM_BUILD_DIR");
    println!("cargo:rerun-if-env-changed=FSIM_LIB_DIR");
    println!("cargo:rerun-if-env-changed=FSIM_BIN_DIR");

    // target/<profile>/ is three levels up from OUT_DIR.
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let target_dir = out.ancestors().nth(3).map(PathBuf::from).unwrap_or(out.clone());
    let mut dlls = vec!["libfsim.dll", "libJSBSim.dll"];
    if vision {
        dlls.push("libfsim_vision.dll");
    }
    for dll in &dlls {
        println!("cargo:rerun-if-changed={}", bin.join(dll).display()); // re-copy after a CMake rebuild
        if !bin.join(dll).exists() {
            println!("cargo:warning={} not found in {}; set FSIM_BIN_DIR or add it to PATH", dll, bin.display());
        }
    }
    // Every DLL in the fsim bin directory: fsim, JSBSim, fsim_vision and, after
    // `cmake --build --target deploy`, all their MSYS2 dependencies.
    if let Ok(entries) = fs::read_dir(&bin) {
        for e in entries.flatten() {
            let p = e.path();
            if p.extension().map(|x| x.eq_ignore_ascii_case("dll")).unwrap_or(false) {
                let _ = fs::copy(&p, target_dir.join(p.file_name().unwrap()));
            }
        }
    }
    // The DLLs are built with MSYS2 UCRT64 GCC and need its runtime; Rust's own MinGW does not ship it.
    // FSIM_RUNTIME_DIR, else the first PATH entry that looks like a ucrt64 bin directory.
    let runtime = ["libstdc++-6.dll", "libgcc_s_seh-1.dll", "libwinpthread-1.dll"];
    let candidates: Vec<PathBuf> = env::var("FSIM_RUNTIME_DIR")
        .map(|d| vec![PathBuf::from(d)])
        .unwrap_or_else(|_| env::split_paths(&env::var_os("PATH").unwrap_or_default()).collect());
    let has_all = |d: &PathBuf| runtime.iter().all(|r| d.join(r).exists());
    let pick = candidates
        .iter()
        .find(|d| d.to_string_lossy().to_lowercase().contains("ucrt64") && has_all(d))
        .or_else(|| candidates.iter().find(|d| has_all(d)));
    match pick {
        Some(dir) => {
            for r in runtime {
                let _ = fs::copy(dir.join(r), target_dir.join(r));
            }
        }
        None => println!("cargo:warning=UCRT64 runtime DLLs not found; set FSIM_RUNTIME_DIR to <msys2>/ucrt64/bin"),
    }
}
