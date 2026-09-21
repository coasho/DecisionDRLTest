# fsim from Rust

The C ABI (`include/fsim/fsim_c.h`) from Rust with nothing but `extern "C"`
declarations: [src/ffi.rs](src/ffi.rs) mirrors the structs field for field,
[src/main.rs](src/main.rs) drives the object model (`world`) and the batch
layer (`vecenv`) - the same PD baseline as `examples/minimal_trainer`, at
the same throughput.

```bash
cd examples/rust_trainer
cargo run --release -- world     # two aircraft at the attitude and velocity levels; start flightsim-viewer.exe to watch
cargo run --release -- vecenv    # 32 environments, PD policy, ~0.7M vehicle-steps/s
```

`build.rs` links `libfsim.dll.a` from `build/ucrt64-release/lib` and copies
`libfsim.dll`, `libJSBSim.dll` and the UCRT64 runtime next to the executable
(`FSIM_BUILD_DIR`, `FSIM_LIB_DIR`, `FSIM_BIN_DIR`, `FSIM_RUNTIME_DIR` override
the locations). Use the `stable-x86_64-pc-windows-gnu` toolchain: the DLLs
are MinGW builds and the import library is a GNU archive.

Cameras work the same way through `fsim_vision_c.h` (link `fsim_vision`).
