# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/esi/code/roboparty_deploy/src/inference/thirdparty/onnxruntime"
  "/home/esi/code/roboparty_deploy/build_sp_d/roboparty_inference/_deps/onnxruntime-build"
  "/home/esi/code/roboparty_deploy/build_sp_d/roboparty_inference/_deps/onnxruntime-subbuild/onnxruntime-populate-prefix"
  "/home/esi/code/roboparty_deploy/build_sp_d/roboparty_inference/_deps/onnxruntime-subbuild/onnxruntime-populate-prefix/tmp"
  "/home/esi/code/roboparty_deploy/build_sp_d/roboparty_inference/_deps/onnxruntime-subbuild/onnxruntime-populate-prefix/src/onnxruntime-populate-stamp"
  "/home/esi/code/roboparty_deploy/build_sp_d/roboparty_inference/_deps/onnxruntime-subbuild/onnxruntime-populate-prefix/src"
  "/home/esi/code/roboparty_deploy/build_sp_d/roboparty_inference/_deps/onnxruntime-subbuild/onnxruntime-populate-prefix/src/onnxruntime-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/esi/code/roboparty_deploy/build_sp_d/roboparty_inference/_deps/onnxruntime-subbuild/onnxruntime-populate-prefix/src/onnxruntime-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/esi/code/roboparty_deploy/build_sp_d/roboparty_inference/_deps/onnxruntime-subbuild/onnxruntime-populate-prefix/src/onnxruntime-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
