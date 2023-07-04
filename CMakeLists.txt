cmake_minimum_required(VERSION 3.13)

project(
    MSPFCI
    VERSION 1.0
    DESCRIPTION "MSPFCI: Multiwii Serial Protocol Flight Controller Interface. Interface Library for flight controllers based on the MultiWii Serial Protocol"
    LANGUAGES CXX
)

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
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=thread -pthread")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=address -pthread")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=undefined -pthread")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -Weffc++ -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread")

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
message(STATUS "Building in: " ${CMAKE_BUILD_TYPE})

# Include header files
include_directories(
    include
)

# List library source files
list(
  APPEND lib_sources
  source/serial/serial.cc
  source/serial/impl/unix.cc
  source/serial/impl/list_ports/list_ports_linux.cc
  source/mspfci/interface.cpp
  source/mspfci/msp.cpp
)

## Declare a C++ library
add_library(mspfci SHARED ${lib_sources})
target_link_libraries(mspfci)

## Declare a C++ executable
add_executable(read_sensors examples/read_sensors.cpp)
target_link_libraries(read_sensors mspfci)
add_executable(read_sensors_async examples/read_sensors_async.cpp)
target_link_libraries(read_sensors_async mspfci)
add_executable(send_commands examples/send_commands.cpp)
target_link_libraries(send_commands mspfci)