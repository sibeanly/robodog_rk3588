# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was roboparty_motorsConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)
find_dependency(fmt)
find_dependency(spdlog)
find_dependency(Eigen3)
find_dependency(Boost COMPONENTS system)

include("${CMAKE_CURRENT_LIST_DIR}/roboparty_motorsTargets.cmake")

if(NOT TARGET roboparty_motors::roboparty_motors)
    add_library(roboparty_motors::roboparty_motors INTERFACE IMPORTED)
    target_link_libraries(roboparty_motors::roboparty_motors INTERFACE 
        roboparty_motors::motors
        roboparty_motors::dm_motors
        roboparty_motors::evo_motors
        roboparty_motors::lro_motors
        roboparty_motors::xyn_motors
        roboparty_motors::motors_can
        roboparty_motors::motors_canfd
    )
endif()

check_required_components(roboparty_motors)
