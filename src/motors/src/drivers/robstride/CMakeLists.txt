# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 Luo1imasi
# Copyright (C) 2026 wentywenty
AUX_SOURCE_DIRECTORY(. SOURCE_LIST_ROBSTRIDE_MOTORS)
add_library(robstride_motors
        STATIC
        ${SOURCE_LIST_ROBSTRIDE_MOTORS}
)
target_include_directories(robstride_motors
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robstride_motors PUBLIC ${PUBLIC_DEPENDENCIES} motors_can motors_canfd)
