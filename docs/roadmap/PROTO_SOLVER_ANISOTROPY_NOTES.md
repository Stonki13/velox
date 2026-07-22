# CPU Anisotropic Friction Convergence Notes

## Finding

The CPU-only Release stress run exposed that an anisotropic box sliding on a
plane retained 0.741 m/s along its high-friction axis after 120 frames. The
same scenario passed through the CUDA path because its graph-colored solver
already executes two velocity sweeps per base iteration.

## Fix

The CPU solver now detects contacts whose two directional friction limits
differ and runs 16 velocity iterations for that step rather than the normal 8.
This is restricted to anisotropic contact sets, where the two tangent rows are
coupled by elliptical Coulomb clamping. Isotropic scenes retain their existing
iteration count and performance profile.

## Verification

The CPU-only Release `stress_demo` now passes all scenarios, including the
anisotropic friction regression. The CUDA-enabled Release build also passed all
14 CTest suites, two `fuzz_demo 80` runs, and `proto_manifold`.

## Merge Recommendation

Ready to merge with the CUDA recovery pass. The additional work is bounded to
an explicitly higher-quality friction case and restores consistent CPU/GPU
acceptance behavior.
