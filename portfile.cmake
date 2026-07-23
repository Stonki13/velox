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
    # SHA-512 of https://github.com/Stonki13/velox/archive/refs/tags/v1.0.0.tar.gz.
    # Update this only after creating and verifying the corresponding signed tag.
    SHA512 8febc652815a3c32acb0e214a66add01c9275198b8abd6e47bfbff400d787d5f1911a50c48fabc8a8a2e5ad78cc383c5a5528b257a069516b2460e634ff5472e
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
