#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cnpy" for configuration "Release"
set_property(TARGET cnpy APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cnpy PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/cnpy/libcnpy.so"
  IMPORTED_SONAME_RELEASE "libcnpy.so"
  )

list(APPEND _cmake_import_check_targets cnpy )
list(APPEND _cmake_import_check_files_for_cnpy "${_IMPORT_PREFIX}/lib/cnpy/libcnpy.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
