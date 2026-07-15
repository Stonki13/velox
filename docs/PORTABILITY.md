# Platform Support

## CPU

Velox's CPU backend is the portable reference backend. It uses C++17 and the
standard threading library, with no Intel- or AMD-specific instruction-set
requirement. Configure a portable build with:

```powershell
cmake -S . -B build-cpu -DVELOX_ENABLE_CUDA=OFF
cmake --build build-cpu --config Release
ctest --test-dir build-cpu -C Release --output-on-failure
```

`BackendType::Auto` selects this backend when the NVIDIA CUDA backend is not
compiled or no CUDA device is present. Applications that require portability
across Intel and AMD systems should select `BackendType::Cpu` explicitly.

## GPU

The current accelerator implementation is NVIDIA CUDA only. AMD and Intel GPUs
run Velox through the CPU backend today. They are not CUDA-compatible and must
not be presented as GPU-accelerated targets.

Supporting their GPUs requires a separate portable accelerator backend with
contact-generation and solver parity tests. The backend interface is designed
for additional implementations, but no HIP, SYCL, OpenCL, or Vulkan backend is
shipped yet.
