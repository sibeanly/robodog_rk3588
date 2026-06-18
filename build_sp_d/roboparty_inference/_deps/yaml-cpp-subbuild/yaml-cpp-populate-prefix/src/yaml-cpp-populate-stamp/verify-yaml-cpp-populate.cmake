# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if("/home/esi/code/roboparty_deploy/src/inference/thirdparty/yaml-cpp-0.9.0.tar.gz" STREQUAL "")
  message(FATAL_ERROR "LOCAL can't be empty")
endif()

if(NOT EXISTS "/home/esi/code/roboparty_deploy/src/inference/thirdparty/yaml-cpp-0.9.0.tar.gz")
  message(FATAL_ERROR "File not found: /home/esi/code/roboparty_deploy/src/inference/thirdparty/yaml-cpp-0.9.0.tar.gz")
endif()

if("SHA256" STREQUAL "")
  message(WARNING "File will not be verified since no URL_HASH specified")
  return()
endif()

if("298593d9c440fd9034b8b193d96318b76d49bc97c6ceadb7b0836edf0b6d7539" STREQUAL "")
  message(FATAL_ERROR "EXPECT_VALUE can't be empty")
endif()

message(STATUS "verifying file...
     file='/home/esi/code/roboparty_deploy/src/inference/thirdparty/yaml-cpp-0.9.0.tar.gz'")

file("SHA256" "/home/esi/code/roboparty_deploy/src/inference/thirdparty/yaml-cpp-0.9.0.tar.gz" actual_value)

if(NOT "${actual_value}" STREQUAL "298593d9c440fd9034b8b193d96318b76d49bc97c6ceadb7b0836edf0b6d7539")
  message(FATAL_ERROR "error: SHA256 hash of
  /home/esi/code/roboparty_deploy/src/inference/thirdparty/yaml-cpp-0.9.0.tar.gz
does not match expected value
  expected: '298593d9c440fd9034b8b193d96318b76d49bc97c6ceadb7b0836edf0b6d7539'
    actual: '${actual_value}'
")
endif()

message(STATUS "verifying file... done")
