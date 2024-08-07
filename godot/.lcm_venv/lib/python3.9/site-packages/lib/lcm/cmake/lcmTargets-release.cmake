#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lcm-static" for configuration "Release"
set_property(TARGET lcm-static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lcm-static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblcm.a"
  )

list(APPEND _cmake_import_check_targets lcm-static )
list(APPEND _cmake_import_check_files_for_lcm-static "${_IMPORT_PREFIX}/lib/liblcm.a" )

# Import target "lcm" for configuration "Release"
set_property(TARGET lcm APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lcm PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblcm.1.5.0.dylib"
  IMPORTED_SONAME_RELEASE "liblcm.1.dylib"
  )

list(APPEND _cmake_import_check_targets lcm )
list(APPEND _cmake_import_check_files_for_lcm "${_IMPORT_PREFIX}/lib/liblcm.1.5.0.dylib" )

# Import target "lcm-gen" for configuration "Release"
set_property(TARGET lcm-gen APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lcm-gen PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/lcm-gen"
  )

list(APPEND _cmake_import_check_targets lcm-gen )
list(APPEND _cmake_import_check_files_for_lcm-gen "${_IMPORT_PREFIX}/bin/lcm-gen" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
