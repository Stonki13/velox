#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "velox::velox" for configuration "Release"
set_property(TARGET velox::velox APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(velox::velox PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvelox.a"
  )

list(APPEND _cmake_import_check_targets velox::velox )
list(APPEND _cmake_import_check_files_for_velox::velox "${_IMPORT_PREFIX}/lib/libvelox.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
