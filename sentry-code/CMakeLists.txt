cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(sentry VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(src test taproot/src taproot/modm/src taproot/modm taproot/sim-modm lib)

file(GLOB_RECURSE sentry_SRC
        "lib/*.h"
        "src/*.hpp"
        "src/*.cpp"
        "test/*.cpp"
        )

add_executable(sentry ${sentry_SRC} test/chassis_subsystem_tests.cpp)
