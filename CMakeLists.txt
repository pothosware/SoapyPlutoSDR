########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8.9)
project(SoapyPlutoSDR CXX)

# select build type to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

find_package(SoapySDR NO_MODULE)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "Soapy SDR development files not found...")
endif ()

########################################################################
# Find libiio
########################################################################
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(LibIIO 0.9 REQUIRED)

if (LibIIO_FOUND)
    message(STATUS "LibIIO_INCLUDE_DIRS: ${LibIIO_INCLUDE_DIRS}")
    message(STATUS "LibIIO_LIBRARIES: ${LibIIO_LIBRARIES}")
    message(STATUS "LibIIO_DEFINITIONS: ${LibIIO_DEFINITIONS}")
endif()

include_directories(${LibIIO_INCLUDE_DIRS})

#enable c++11 features
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")

    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wc++11-extensions")
    endif()

    #Thread support enabled (not the same as -lpthread)
    list(APPEND LibIIO_LIBRARIES -pthread)

    #disable warnings for unused parameters
    add_compile_options(-Wall -Wextra -pedantic -Wno-unused-parameter)
endif()

########################################################################
# Find libad9361
########################################################################
find_package(LibAD9361)

if (LibAD9361_FOUND)
    message(STATUS "LibAD9361_INCLUDE_DIRS: ${LibAD9361_INCLUDE_DIRS}")
    message(STATUS "LibAD9361_LIBRARIES: ${LibAD9361_LIBRARIES}")
    message(STATUS "LibAD9361_DEFINITIONS: ${LibAD9361_DEFINITIONS}")
    include_directories(${LibAD9361_INCLUDE_DIRS})
    add_definitions(-DHAS_AD9361_IIO)
endif()

########################################################################
# Find libusb
########################################################################
find_package(LibUSB)

if(LibUSB_FOUND)
    message(STATUS "LibUSB_INCLUDE_DIRS: ${LibUSB_INCLUDE_DIRS}")
    message(STATUS "LibUSB_LIBRARIES: ${LibUSB_LIBRARIES}")
    message(STATUS "LibUSB_DEFINITIONS: ${LibUSB_DEFINITIONS}")
    include_directories(${LibUSB_INCLUDE_DIRS})
    add_definitions(-DHAS_LIBUSB1)
    #disable warnings for libusb.h
    add_compile_options($<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wno-zero-length-array>)
endif()


SOAPY_SDR_MODULE_UTIL(
    TARGET PlutoSDRSupport
    SOURCES
    PlutoSDR_Registration.cpp
    PlutoSDR_Settings.cpp
    PlutoSDR_Streaming.cpp
    LIBRARIES ${LibIIO_LIBRARIES} ${LibAD9361_LIBRARIES} ${LibUSB_LIBRARIES}
)

########################################################################
# uninstall target
########################################################################
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
