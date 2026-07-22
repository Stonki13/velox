# vcpkg port for Velox. Together with the sibling vcpkg.json this directory is a
# valid overlay/registry port: `vcpkg install velox --overlay-ports=<this dir>`
# builds and installs the library, public headers, and the CMake package config
# consumed via find_package(Velox).
#
# Static vs shared is controlled by the active triplet's VCPKG_LIBRARY_LINKAGE;
# vcpkg_cmake_configure forwards BUILD_SHARED_LIBS accordingly and the project's
# CMakeLists defines VELOX_BUILDING_SHARED / VELOX_USING_SHARED from the resolved
# target type, so both linkages export/import symbols correctly.

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO Stonki13/velox
    REF "v${VERSION}"
    # TODO(maintainer): replace with the SHA-512 of the v${VERSION} source
    # archive produced by the Release workflow before publishing the port.
    SHA512 0
    HEAD_REF main
)

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        cuda VELOX_ENABLE_CUDA
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${FEATURE_OPTIONS}
        -DVELOX_BUILD_EXAMPLES=OFF
        -DVELOX_BUILD_SANDBOX=OFF
        -DVELOX_BUILD_DIFFTEST=OFF
        -DBUILD_TESTING=OFF
)

vcpkg_cmake_install()

# Relocate the installed CMake package (lib/cmake/Velox) into share/velox and
# fix up the imported target paths so find_package(Velox CONFIG) works for
# consumers in both static and shared triplets.
vcpkg_cmake_config_fixup(PACKAGE_NAME Velox CONFIG_PATH lib/cmake/Velox)
vcpkg_fixup_pkgconfig()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
