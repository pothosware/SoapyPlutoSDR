# - Try to find libiio
# Once done this will define
#
#  LibIIO_FOUND - system has libiio
#  LibIIO_INCLUDE_DIRS - the libiio include directory
#  LibIIO_LIBRARIES - Link these to use libiio
#  LibIIO_DEFINITIONS - Compiler switches required for using libiio
#  LibIIO_VERSION - the libiio version
#
# Redistribution and use is allowed according to the terms of the New BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

find_package(PkgConfig)
pkg_check_modules(PC_LibIIO QUIET libiio)
set(LibIIO_DEFINITIONS ${PC_LibIIO_CFLAGS_OTHER})

# $ENV{HOMEBREW_PREFIX} can be /opt/homebrew (arm64) or defaults to /usr/local
set(HOMEBREW_PREFIX $ENV{HOMEBREW_PREFIX})
if(NOT HOMEBREW_PREFIX)
    set(HOMEBREW_PREFIX "/usr/local")
endif()

find_path(LibIIO_INCLUDE_DIR iio.h
          HINTS ${PC_LibIIO_INCLUDEDIR} ${PC_LibIIO_INCLUDE_DIRS}
          PATHS
          /opt/local/Library/Frameworks
          ${HOMEBREW_PREFIX}/Cellar/libiio/${PC_LibIIO_VERSION}/Frameworks
          ${HOMEBREW_PREFIX}/Cellar/libiio/${PC_LibIIO_VERSION}
          PATH_SUFFIXES libiio)

find_library(LibIIO_LIBRARY NAMES iio libiio
             HINTS ${PC_LibIIO_LIBDIR} ${PC_LibIIO_LIBRARY_DIRS}
             PATHS
             /opt/local/Library/Frameworks
             ${HOMEBREW_PREFIX}/Cellar/libiio/${PC_LibIIO_VERSION}/Frameworks
             ${HOMEBREW_PREFIX}/Cellar/libiio/${PC_LibIIO_VERSION})

set(LibIIO_VERSION ${PC_LibIIO_VERSION})

include(FindPackageHandleStandardArgs)
# Note that `FOUND_VAR LibIIO_FOUND` is needed for cmake 3.2 and older.
find_package_handle_standard_args(LibIIO
                                  FOUND_VAR LibIIO_FOUND
                                  REQUIRED_VARS LibIIO_LIBRARY LibIIO_INCLUDE_DIR
                                  VERSION_VAR LibIIO_VERSION)

mark_as_advanced(LibIIO_INCLUDE_DIR LibIIO_LIBRARY)

set(LibIIO_LIBRARIES ${LibIIO_LIBRARY})
set(LibIIO_INCLUDE_DIRS ${LibIIO_INCLUDE_DIR})
