########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8.9)
project(SoapyPlutoSDR CXX)

find_package(SoapySDR NO_MODULE)
 if (NOT SoapySDR_FOUND) 
    message(FATAL_ERROR "Soapy SDR development files not found...") 
 endif () 

########################################################################
# Find libiio
########################################################################
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(libiio 0.9 REQUIRED)

if (LIBIIO_FOUND)
    message(STATUS "LIBIIO_INCLUDE_DIRS: ${LIBIIO_INCLUDE_DIRS}")
    message(STATUS "LIBIIO_LIBRARIES: ${LIBIIO_LIBRARIES}")
    message(STATUS "LIBIIO_DEFINITIONS: ${LIBIIO_DEFINITIONS}")
endif()

include_directories(${LIBIIO_INCLUDE_DIRS})

find_package(Threads)

#enable c++11 features
if(CMAKE_COMPILER_IS_GNUCXX)

    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    endif()

    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)

endif(CMAKE_COMPILER_IS_GNUCXX)


SOAPY_SDR_MODULE_UTIL(
    TARGET PlutSDRSupport
    SOURCES
    PlutoSDR_Registation.cpp
    PlutoSDR_Settings.cpp
    PlutoSDR_Streaming.cpp
    LIBRARIES ${LIBIIO_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}
)
