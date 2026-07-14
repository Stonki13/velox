# 29 — Community Infrastructure

## Goal

Establish issue templates, a contribution guide, and polished example projects that lower the barrier to entry for contributors and users. A well-documented community process accelerates bug reporting, feature requests, and external contributions.

## Public API

```cpp
// No new public symbols. This item adds documentation, templates, and examples.
// All changes are in docs/, .github/, and examples/.

// Issue template fields (for GitHub issue form):
//   Velox version: [e.g., 0.9.0]
//   Platform: [Windows/Linux/macOS, x86_64/ARM]
//   Backend: [CPU/CUDA/Auto]
//   Reproduction steps: [numbered list]
//   Expected behavior: [what should happen]
//   Actual behavior: [what actually happens]
//   Repro scene: [attach a serialized scene if applicable]
//   Logs/outputs: [StepStats, debug lines, crash dumps]

```

## Data structures

- No new library data structures. All changes are in documentation and template files.

## Algorithm

**Issue templates:**

1. **Bug report template (`.github/ISSUE_TEMPLATE/bug_report.md`):**
   - Requests the user to provide: Velox version, platform, backend type, reproduction steps, expected vs actual behavior, and a repro scene (serialized via `serializeWorld()`).
   - The repro scene request is critical: without a scene, debugging tunneling or stacking issues is nearly impossible.
2. **Feature request template (`.github/ISSUE_TEMPLATE/feature_request.md`):**
   - Asks for: description of the feature, use case, proposed API design (optional), and links to similar features in other engines (Jolt, Bullet, PhysX).
3. **Performance issue template (`.github/ISSUE_TEMPLATE/performance_issue.md`):**
   - Requests: scene description, StepStats output, comparison with expected performance, and hardware specs.

**Contribution guide (`docs/CONTRIBUTING.md`):**

1. **Code style:** Follow the existing codebase conventions (4-space indent, snake_case for functions, CamelCase for types, VELOX_HD macros for device-compatible code).
2. **Testing:** All new features must include tests in `tests/`. Run the full test suite (`ctest --test-dir build`) before submitting a PR.
3. **Performance:** New code must not regress benchmark results by > 5%. Run `runBenchmark()` on canonical scenes and compare against baselines.
4. **CUDA compatibility:** Any new math or geometry code must be VELOX_HD-compatible (compiles with `__host__ __device__`). Test on both CPU and CUDA backends.
5. **PR process:** Fork the repo, create a feature branch, commit with descriptive messages, push to the fork, open a PR against `main`. Require at least one maintainer review before merge.

**Polished examples (`examples/`):**

1. **`hello_velox.cpp`:** Minimal "add a sphere, step the world" example. Compiles with a single CMake command.
2. **`contact_events.cpp`:** Demonstrates contact Begin/Persist/End events and sensor bodies.
3. **`joints_demo.cpp`:** Shows all joint types (hinge, spring, cone-twist, 6DoF) in action.
4. **`queries.cpp`:** Raycast, overlap, and shape cast examples with debug line output.
5. **`multi_threaded.cpp`:** Demonstrates cross-thread queries with ThreadSafetyPolicy::Relaxed.

## Files

- `.github/ISSUE_TEMPLATE/bug_report.md` — new bug report template
- `.github/ISSUE_TEMPLATE/feature_request.md` — new feature request template
- `.github/ISSUE_TEMPLATE/performance_issue.md` — new performance issue template
- `docs/CONTRIBUTING.md` — contribution guide
- `examples/hello_velox.cpp` — minimal example
- `examples/contact_events.cpp` — contact events demo
- `examples/joints_demo.cpp` — joints demonstration
- `examples/queries.cpp` — query API examples

## Tests

1. **Issue template rendering:** Render each issue template on GitHub and verify that all fields are present and properly formatted.
2. **Example compilation:** Compile all 5 example projects; verify they link against Velox and run without errors.
3. **Contribution guide accuracy:** Follow the contribution guide's steps to set up a development environment; verify that the process works as documented (clone, cmake, build, test).

## Acceptance

- [ ] Bug report template requests a repro scene attachment
- [ ] Contribution guide documents code style, testing requirements, and PR process
- [ ] All 5 example projects compile and run successfully
- [ ] `hello_velox.cpp` is self-contained (no external dependencies beyond Velox)
- [ ] Issue templates are linked from the repository's main README

## Size: S

## Risks

- Example projects can become outdated if the API changes. Must add a CI check that compiles all examples on every PR; fail the build if any example doesn't compile.
- Contribution guides that are too detailed discourage contributors; too vague leads to inconsistent code quality. Strike a balance: document the non-negotiable rules (testing, CUDA compatibility) and leave style decisions to existing conventions.
- Issue templates that request too much information may reduce the number of bug reports. Keep the repro scene request optional but strongly encouraged; provide a helper function (`velox::test::serializeWorld()`) that makes it easy for users to attach scenes.
