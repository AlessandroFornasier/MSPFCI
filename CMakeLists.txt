cmake_minimum_required(VERSION 3.11)

project(
    MSPFCI
    VERSION 1.0
    DESCRIPTION "MSPFCI: Multiwii Serial Protocol Flight Controller Interface. Interface Library for flight controllers based on the MultiWii Serial Protocol"
    LANGUAGES CXX
)

# Include libraries
find_package(serial REQUIRED)

# display message to user
message(STATUS "SERIAL VERSION: " ${serial_VERSION})

# Set compiler, try to compile with newer version of C++
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX20)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
    set(CMAKE_CXX_STANDARD 20)
elseif(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    set(CMAKE_CXX_STANDARD 17)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} do not support required version. Please use a different C++ compiler.")
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "Set compiler ${CMAKE_CXX_COMPILER}.")

# Optimization flags
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=address")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Define build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  message(STATUS "UNDEFINED CMAKE_BUILD_TYPE, BUILDING IN RELEASE")
  message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
else()
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
    elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
        message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
    endif()
endif ()

# Include header files
include_directories(
    include
    ${serial_INCLUDE_DIR}
)

# Set link libraries used by all binaries
list(
    APPEND thirdparty_libraries
    ${serial_LIBRARIES}
)

# List library source files
list(
  APPEND lib_sources
  source/interface.cpp
)

## Declare a C++ library
add_library(mspfci SHARED ${lib_sources})
target_link_libraries(mspfci ${thirdparty_libraries})

## Declare a C++ executable
add_executable(read_sensors examples/read_sensors.cpp)
target_link_libraries(read_sensors ${thirdparty_libraries} mspfci)