# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if("/home/esi/code/roboparty_deploy/src/inference/thirdparty/onnxruntime-linux-aarch64-1.21.0.tgz" STREQUAL "")
  message(FATAL_ERROR "LOCAL can't be empty")
endif()

if(NOT EXISTS "/home/esi/code/roboparty_deploy/src/inference/thirdparty/onnxruntime-linux-aarch64-1.21.0.tgz")
  message(FATAL_ERROR "File not found: /home/esi/code/roboparty_deploy/src/inference/thirdparty/onnxruntime-linux-aarch64-1.21.0.tgz")
endif()

if("SHA256" STREQUAL "")
  message(WARNING "File will not be verified since no URL_HASH specified")
  return()
endif()

if("4508084bde1232ee1ab4b6fad2155be0ea2ccab1c1aae9910ddb3fb68a60805e" STREQUAL "")
  message(FATAL_ERROR "EXPECT_VALUE can't be empty")
endif()

message(STATUS "verifying file...
     file='/home/esi/code/roboparty_deploy/src/inference/thirdparty/onnxruntime-linux-aarch64-1.21.0.tgz'")

file("SHA256" "/home/esi/code/roboparty_deploy/src/inference/thirdparty/onnxruntime-linux-aarch64-1.21.0.tgz" actual_value)

if(NOT "${actual_value}" STREQUAL "4508084bde1232ee1ab4b6fad2155be0ea2ccab1c1aae9910ddb3fb68a60805e")
  message(FATAL_ERROR "error: SHA256 hash of
  /home/esi/code/roboparty_deploy/src/inference/thirdparty/onnxruntime-linux-aarch64-1.21.0.tgz
does not match expected value
  expected: '4508084bde1232ee1ab4b6fad2155be0ea2ccab1c1aae9910ddb3fb68a60805e'
    actual: '${actual_value}'
")
endif()

message(STATUS "verifying file... done")
