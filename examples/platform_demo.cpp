#include <velox/platform.h>

#include <cstdio>

int main() {
    const int platforms = VELOX_PLATFORM_WINDOWS + VELOX_PLATFORM_LINUX +
                          VELOX_PLATFORM_MACOS;
    const int architectures = VELOX_ARCH_X86_64 + VELOX_ARCH_ARM64;
    const int platformArchitectures = VELOX_PLATFORM_MACOS_X86 +
                                      VELOX_PLATFORM_MACOS_ARM +
                                      VELOX_PLATFORM_LINUX_X86 +
                                      VELOX_PLATFORM_LINUX_ARM;
    const bool ok = platforms == 1 && architectures == 1 &&
                    (VELOX_PLATFORM_WINDOWS || platformArchitectures == 1);
    std::printf("platform_demo: os=%d arch=%d specific=%d %s\n", platforms,
                architectures, platformArchitectures, ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
