#pragma once

#include <cstdint>

namespace velox {

// Semantic version of the public Velox API. CMake reads these constants when
// configuring the package, so installed package metadata cannot drift from the
// headers consumers compile against.
inline constexpr uint32_t kVersionMajor = 0;
inline constexpr uint32_t kVersionMinor = 1;
inline constexpr uint32_t kVersionPatch = 0;
inline constexpr const char kVersionString[] = "0.1.0";
inline constexpr uint32_t kMinCxxStandard = 17;

inline constexpr const char* versionString() { return kVersionString; }

} // namespace velox

#if defined(_MSC_VER)
#define VELOX_DEPRECATED(message) __declspec(deprecated(message))
#elif defined(__GNUC__) || defined(__clang__)
#define VELOX_DEPRECATED(message) __attribute__((deprecated(message)))
#else
#define VELOX_DEPRECATED(message)
#endif
