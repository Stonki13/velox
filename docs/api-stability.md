# API Stability Policy

This document defines the versioning, deprecation, and ABI stability
guarantees for the Velox physics engine public API.

---

## 1. Versioning Policy

Velox follows [Semantic Versioning 2.0.0](https://semver.org/):

| Component | Bumped when… |
|-----------|-------------|
| **MAJOR** | Backward-incompatible API or ABI changes are made. |
| **MINOR** | New functionality is added in a backward-compatible manner. |
| **PATCH** | Backward-compatible bug fixes are applied. |

### Current version

The canonical version constants live in `include/velox/version.h`:

```cpp
inline constexpr uint32_t kVersionMajor = 1;
inline constexpr uint32_t kVersionMinor = 0;
inline constexpr uint32_t kVersionPatch = 0;
```

Preprocessor equivalents are in `include/velox/api_version.h`:

```cpp
#define VELOX_VERSION_MAJOR 1
#define VELOX_VERSION_MINOR 0
#define VELOX_VERSION_PATCH 0
```

### Version encoding

A single comparable integer is produced by:

```cpp
#define VELOX_VERSION_ENCODE(major, minor, patch) \
    (((major) * 10000u) + ((minor) * 100u) + (patch))
```

Use `VELOX_REQUIRE_VERSION(major, minor, patch)` at file scope to fail the
build when headers are too old.

---

## 2. ABI Stability Guarantees

### What is stable

Starting with version **1.0.0**, the following are ABI-stable within a major
version series:

- **C API function signatures** in `velox_c.h` — existing functions keep
  their exact parameter types and return types.
- **`velox_Api` vtable layout** in `stable_api.h` — fields are append-only;
  existing entries never move or change type.
- **Public struct sizes and member offsets** for types annotated with
  `VELOX_ABI_CHECK_*` assertions in `api_version.h`.
- **Enum underlying values** — enumerators keep their numeric values; new
  enumerators are appended.

### What is NOT stable

- C++ class layouts (vtable order, private members).
- Internal headers under `src/`.
- CUDA device code interfaces.
- Serialization binary format (use the versioned v2 format for persistence).

### Compile-time ABI checks

Consumers can embed layout assertions in their own code:

```cpp
#include <velox/api_version.h>
#include <velox/velox_c.h>

VELOX_ABI_CHECK_SIZE(velox_Vec3, 12);
VELOX_ABI_CHECK_ALIGN(velox_Vec3, 4);
VELOX_ABI_CHECK_OFFSET(velox_RayHit, distance, 12);
```

If any assertion fires at compile time, the ABI has broken and the library
must be rebuilt or the consumer updated.

---

## 3. Deprecation Timeline

| Phase | Duration | What happens |
|-------|----------|-------------|
| **Announcement** | Current minor release | Symbol is annotated with `VELOX_DEPRECATED_SINCE` or `VELOX_REMOVAL_IN`. Compiles with a warning. |
| **Grace period** | ≥ 2 minor releases | Deprecated symbol remains fully functional. Warnings guide migration. |
| **Removal** | Next major release | Symbol is deleted. Code using it fails to compile. |

### Deprecation macros

```cpp
// Mark deprecated since a specific version:
VELOX_DEPRECATED_SINCE(1, 1, "use World::stepFixed() instead")
void step(float dt);

// Mark for removal in a future major:
VELOX_REMOVAL_IN(2, 0, "migrate to serialization v2")
void serializeLegacy(Buffer& buf);
```

### Current deprecations

| Symbol | Deprecated since | Removal target | Replacement |
|--------|-----------------|---------------|-------------|
| *(none yet)* | — | — | — |

All removals are recorded in `CHANGELOG.md`.

---

## 4. Feature Detection

Compile-time feature macros let consumers conditionally compile optional
modules:

```cpp
#include <velox/api_version.h>

#if VELOX_FEATURE(CUDA)
    // CUDA-accelerated path
#endif

#if VELOX_FEATURE(REPLAY)
    // Deterministic replay recording
#endif
```

Available feature macros:

| Macro | Meaning |
|-------|---------|
| `VELOX_HAS_CUDA` | CUDA backend compiled in |
| `VELOX_HAS_REPLAY` | Deterministic replay system |
| `VELOX_HAS_CHARACTER` | Character controller |
| `VELOX_HAS_VEHICLE` | Vehicle module |
| `VELOX_HAS_RAGDOLL` | Ragdoll module |
| `VELOX_HAS_SERIALIZATION_V2` | V2 serialization format |
| `VELOX_HAS_STABLE_C_API` | Versioned stable C API |
| `VELOX_HAS_CCD` | Continuous collision detection |
| `VELOX_HAS_DEBUG_DRAW` | Debug draw interface |

---

## 5. Stable C API (`stable_api.h`)

The versioned vtable in `stable_api.h` is the recommended integration point
for:

- Dynamic loading / plugin architectures
- Foreign-language bindings (C#, Python, Rust, Go)
- Applications that must survive library upgrades without recompilation

### Usage

```c
#include <velox/stable_api.h>

velox_Api api;
velox_Api_Init(&api);

if (!velox_Api_IsCompatible(&api, VELOX_VERSION_ENCODE(1, 0, 0))) {
    // handle incompatibility
}

velox_World* w = api.World_Create(VELOX_BACKEND_CPU);
api.World_Step(w, 1.0f / 60.0f);
api.World_Destroy(w);
```

### Backward-compatibility shims

Legacy call signatures are preserved as inline `_Legacy` wrappers:

```c
// Old: int backend selector
velox_World* w = velox_World_Create_Legacy(1);

// Old: double timestep
velox_World_Step_Legacy(w, 1.0 / 60.0);

// Old: three-float gravity
velox_World_SetGravity_Legacy(w, 0.0f, -9.81f, 0.0f);
```

These shims carry deprecation warnings and will be removed in the next major
version.

---

## 6. Migration Guides

### Migrating from pre-1.0 (0.x) to 1.0

1. **Replace raw version checks** with `VELOX_REQUIRE_VERSION(1, 0, 0)`.
2. **Switch to the stable C API** if you use `velox_c.h` from a dynamic
   loader — the vtable gives you runtime version negotiation.
3. **Remove workarounds** for pre-1.0 struct layout changes; the layout is
   now frozen for the 1.x series.
4. **Update deprecation suppressions** — any `VELOX_DEPRECATED` warnings
   from 0.x headers that you silenced should be re-evaluated.

### Migrating from 1.0 to 1.x (minor updates)

- Minor releases are source- and binary-compatible. Recompile to pick up
  new features; existing binaries continue to work.
- New vtable entries are appended; old binaries that use a smaller
  `velox_Api` struct are unaffected (trailing bytes are zeroed by
  `velox_Api_Init`).

### Migrating to 2.0 (future major)

1. **Address all deprecation warnings** before upgrading. Every symbol
   removed in 2.0 will have been annotated with `VELOX_REMOVAL_IN(2, 0, …)`
   for at least two prior minor releases.
2. **Re-run ABI checks** — `VELOX_ABI_CHECK_*` assertions will flag any
   struct layout changes.
3. **Update feature detection** — some `VELOX_HAS_*` macros may change
   default values if modules are reorganized.
4. **Rebuild all binaries** — ABI is not guaranteed across major versions.

---

## 7. Support Window

| Version | Status | End of support |
|---------|--------|---------------|
| 1.0.x | Active | — |
| 0.x | EOL | No further patches |

Security and correctness fixes are backported to the current minor release
for the lifetime of the major version.
