#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "roboparty_motors::motors" for configuration "Release"
set_property(TARGET roboparty_motors::motors APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_motors::motors PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmotors.a"
  )

list(APPEND _cmake_import_check_targets roboparty_motors::motors )
list(APPEND _cmake_import_check_files_for_roboparty_motors::motors "${_IMPORT_PREFIX}/lib/libmotors.a" )

# Import target "roboparty_motors::dm_motors" for configuration "Release"
set_property(TARGET roboparty_motors::dm_motors APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_motors::dm_motors PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdm_motors.a"
  )

list(APPEND _cmake_import_check_targets roboparty_motors::dm_motors )
list(APPEND _cmake_import_check_files_for_roboparty_motors::dm_motors "${_IMPORT_PREFIX}/lib/libdm_motors.a" )

# Import target "roboparty_motors::evo_motors" for configuration "Release"
set_property(TARGET roboparty_motors::evo_motors APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_motors::evo_motors PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libevo_motors.a"
  )

list(APPEND _cmake_import_check_targets roboparty_motors::evo_motors )
list(APPEND _cmake_import_check_files_for_roboparty_motors::evo_motors "${_IMPORT_PREFIX}/lib/libevo_motors.a" )

# Import target "roboparty_motors::lro_motors" for configuration "Release"
set_property(TARGET roboparty_motors::lro_motors APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_motors::lro_motors PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblro_motors.a"
  )

list(APPEND _cmake_import_check_targets roboparty_motors::lro_motors )
list(APPEND _cmake_import_check_files_for_roboparty_motors::lro_motors "${_IMPORT_PREFIX}/lib/liblro_motors.a" )

# Import target "roboparty_motors::robstride_motors" for configuration "Release"
set_property(TARGET roboparty_motors::robstride_motors APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_motors::robstride_motors PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librobstride_motors.a"
  )

list(APPEND _cmake_import_check_targets roboparty_motors::robstride_motors )
list(APPEND _cmake_import_check_files_for_roboparty_motors::robstride_motors "${_IMPORT_PREFIX}/lib/librobstride_motors.a" )

# Import target "roboparty_motors::xyn_motors" for configuration "Release"
set_property(TARGET roboparty_motors::xyn_motors APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_motors::xyn_motors PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libxyn_motors.a"
  )

list(APPEND _cmake_import_check_targets roboparty_motors::xyn_motors )
list(APPEND _cmake_import_check_files_for_roboparty_motors::xyn_motors "${_IMPORT_PREFIX}/lib/libxyn_motors.a" )

# Import target "roboparty_motors::motors_can" for configuration "Release"
set_property(TARGET roboparty_motors::motors_can APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_motors::motors_can PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmotors_can.a"
  )

list(APPEND _cmake_import_check_targets roboparty_motors::motors_can )
list(APPEND _cmake_import_check_files_for_roboparty_motors::motors_can "${_IMPORT_PREFIX}/lib/libmotors_can.a" )

# Import target "roboparty_motors::motors_canfd" for configuration "Release"
set_property(TARGET roboparty_motors::motors_canfd APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboparty_motors::motors_canfd PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmotors_canfd.a"
  )

list(APPEND _cmake_import_check_targets roboparty_motors::motors_canfd )
list(APPEND _cmake_import_check_files_for_roboparty_motors::motors_canfd "${_IMPORT_PREFIX}/lib/libmotors_canfd.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
