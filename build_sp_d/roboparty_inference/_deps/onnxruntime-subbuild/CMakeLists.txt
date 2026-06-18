# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.28.3)

# Reject any attempt to use a toolchain file. We must not use one because
# we could be downloading it here. If the CMAKE_TOOLCHAIN_FILE environment
# variable is set, the cache variable will have been initialized from it.
unset(CMAKE_TOOLCHAIN_FILE CACHE)
unset(ENV{CMAKE_TOOLCHAIN_FILE})

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(onnxruntime-populate NONE)



include(ExternalProject)
ExternalProject_Add(onnxruntime-populate
                     "UPDATE_DISCONNECTED" "False" "DOWNLOAD_EXTRACT_TIMESTAMP" "TRUE" "EXTERNALPROJECT_INTERNAL_ARGUMENT_SEPARATOR" "URL" "/home/esi/code/roboparty_deploy/src/inference/thirdparty/onnxruntime-linux-aarch64-1.21.0.tgz" "URL_HASH" "SHA256=4508084bde1232ee1ab4b6fad2155be0ea2ccab1c1aae9910ddb3fb68a60805e"
                    SOURCE_DIR          "/home/esi/code/roboparty_deploy/src/inference/thirdparty/onnxruntime"
                    BINARY_DIR          "/home/esi/code/roboparty_deploy/build_sp_d/roboparty_inference/_deps/onnxruntime-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
                    USES_TERMINAL_PATCH     YES
)


