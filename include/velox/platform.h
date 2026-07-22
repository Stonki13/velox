#pragma once

// Public compile-time platform identifiers. Every macro is defined as 0 or 1
// so consumers can use them in ordinary preprocessor expressions.
#define VELOX_PLATFORM_WINDOWS 0
#define VELOX_PLATFORM_LINUX 0
#define VELOX_PLATFORM_MACOS 0
#define VELOX_ARCH_X86_64 0
#define VELOX_ARCH_ARM64 0
#define VELOX_PLATFORM_MACOS_X86 0
#define VELOX_PLATFORM_MACOS_ARM 0
#define VELOX_PLATFORM_LINUX_X86 0
#define VELOX_PLATFORM_LINUX_ARM 0

#if defined(_WIN32)
#undef VELOX_PLATFORM_WINDOWS
#define VELOX_PLATFORM_WINDOWS 1
#elif defined(__APPLE__)
#undef VELOX_PLATFORM_MACOS
#define VELOX_PLATFORM_MACOS 1
#elif defined(__linux__)
#undef VELOX_PLATFORM_LINUX
#define VELOX_PLATFORM_LINUX 1
#endif

#if defined(_M_X64) || defined(__x86_64__)
#undef VELOX_ARCH_X86_64
#define VELOX_ARCH_X86_64 1
#elif defined(_M_ARM64) || defined(__aarch64__)
#undef VELOX_ARCH_ARM64
#define VELOX_ARCH_ARM64 1
#endif

#if VELOX_PLATFORM_MACOS && VELOX_ARCH_X86_64
#undef VELOX_PLATFORM_MACOS_X86
#define VELOX_PLATFORM_MACOS_X86 1
#elif VELOX_PLATFORM_MACOS && VELOX_ARCH_ARM64
#undef VELOX_PLATFORM_MACOS_ARM
#define VELOX_PLATFORM_MACOS_ARM 1
#elif VELOX_PLATFORM_LINUX && VELOX_ARCH_X86_64
#undef VELOX_PLATFORM_LINUX_X86
#define VELOX_PLATFORM_LINUX_X86 1
#elif VELOX_PLATFORM_LINUX && VELOX_ARCH_ARM64
#undef VELOX_PLATFORM_LINUX_ARM
#define VELOX_PLATFORM_LINUX_ARM 1
#endif
