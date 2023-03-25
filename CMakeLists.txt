# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
# set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g -std=c++17")
# set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -std=c++20")
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g -std=c++17")
message($ENV{IDF_PATH})
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(main)
