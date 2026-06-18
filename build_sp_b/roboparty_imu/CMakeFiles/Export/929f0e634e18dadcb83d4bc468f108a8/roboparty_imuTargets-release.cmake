#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "roboparty_imu::imu" for configuration "Release"
set_property(TARGET roboparty_imu::imu APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_imu::imu PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libimu.a"
  )

list(APPEND _cmake_import_check_targets roboparty_imu::imu )
list(APPEND _cmake_import_check_files_for_roboparty_imu::imu "${_IMPORT_PREFIX}/lib/libimu.a" )

# Import target "roboparty_imu::hipnuc_imu" for configuration "Release"
set_property(TARGET roboparty_imu::hipnuc_imu APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_imu::hipnuc_imu PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C;CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhipnuc_imu.a"
  )

list(APPEND _cmake_import_check_targets roboparty_imu::hipnuc_imu )
list(APPEND _cmake_import_check_files_for_roboparty_imu::hipnuc_imu "${_IMPORT_PREFIX}/lib/libhipnuc_imu.a" )

# Import target "roboparty_imu::damiao_imu" for configuration "Release"
set_property(TARGET roboparty_imu::damiao_imu APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_imu::damiao_imu PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdamiao_imu.a"
  )

list(APPEND _cmake_import_check_targets roboparty_imu::damiao_imu )
list(APPEND _cmake_import_check_files_for_roboparty_imu::damiao_imu "${_IMPORT_PREFIX}/lib/libdamiao_imu.a" )

# Import target "roboparty_imu::imu_protocol" for configuration "Release"
set_property(TARGET roboparty_imu::imu_protocol APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_imu::imu_protocol PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libimu_protocol.a"
  )

list(APPEND _cmake_import_check_targets roboparty_imu::imu_protocol )
list(APPEND _cmake_import_check_files_for_roboparty_imu::imu_protocol "${_IMPORT_PREFIX}/lib/libimu_protocol.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
