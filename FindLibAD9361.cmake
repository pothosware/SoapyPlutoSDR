# - Try to find libad9361-iio
# Once done this will define
#
#  LibAD9361_FOUND - system has libad9361
#  LibAD9361_INCLUDE_DIRS - the libad9361 include directory
#  LibAD9361_LIBRARIES - Link these to use libad9361
#  LibAD9361_DEFINITIONS - Compiler switches required for using libad9361
#  LibAD9361_VERSION - the libad9361 version
#
# Redistribution and use is allowed according to the terms of the New BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

find_package(PkgConfig)
pkg_check_modules(PC_LibAD9361 QUIET libad9361)
set(LibAD9361_DEFINITIONS ${PC_LibAD9361_CFLAGS_OTHER})

# $ENV{HOMEBREW_PREFIX} can be /opt/homebrew (arm64) or defaults to /usr/local
set(HOMEBREW_PREFIX $ENV{HOMEBREW_PREFIX})
if(NOT HOMEBREW_PREFIX)
    set(HOMEBREW_PREFIX "/usr/local")
endif()

# Note: a 0.2 version might be named 0.3 and a 0.1 might be missing the version
find_path(LibAD9361_INCLUDE_DIR ad9361.h
          HINTS ${PC_LibAD9361_INCLUDEDIR} ${PC_LibAD9361_INCLUDE_DIRS}
          PATHS
          /opt/local/Library/Frameworks
          ${HOMEBREW_PREFIX}/Cellar/libad9361-iio/${PC_LibIIO_VERSION}/Frameworks
          ${HOMEBREW_PREFIX}/Cellar/libad9361-iio/0.3/Frameworks
          ${HOMEBREW_PREFIX}/Cellar/libad9361/${PC_LibIIO_VERSION}
          ${HOMEBREW_PREFIX}/Cellar/libad9361/0.1
          PATH_SUFFIXES libad9361-iio)

find_library(LibAD9361_LIBRARY NAMES ad9361 libad9361
             HINTS ${PC_LibAD9361_LIBDIR} ${PC_LibAD9361_LIBRARY_DIRS}
             PATHS
             /opt/local/Library/Frameworks
             ${HOMEBREW_PREFIX}/Cellar/libad9361-iio/${PC_LibIIO_VERSION}/Frameworks
             ${HOMEBREW_PREFIX}/Cellar/libad9361-iio/0.3/Frameworks
             ${HOMEBREW_PREFIX}/Cellar/libad9361/${PC_LibIIO_VERSION}
             ${HOMEBREW_PREFIX}/Cellar/libad9361/0.1)

set(LibAD9361_VERSION ${PC_LibAD9361_VERSION})

include(FindPackageHandleStandardArgs)
# Note that `FOUND_VAR LibIIO_FOUND` is needed for cmake 3.2 and older.
find_package_handle_standard_args(LibAD9361
                                  FOUND_VAR LibAD9361_FOUND
                                  REQUIRED_VARS LibAD9361_LIBRARY LibAD9361_INCLUDE_DIR
                                  VERSION_VAR LibAD9361_VERSION)

mark_as_advanced(LibAD9361_INCLUDE_DIR LibAD9361_LIBRARY)

if (LibAD9361_FOUND)
set(LibAD9361_LIBRARIES ${LibAD9361_LIBRARY})
set(LibAD9361_INCLUDE_DIRS ${LibAD9361_INCLUDE_DIR})
endif()
