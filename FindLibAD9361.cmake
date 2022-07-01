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

find_path(LibAD9361_INCLUDE_DIR ad9361.h
          HINTS ${PC_LibAD9361_INCLUDEDIR} ${PC_LibAD9361_INCLUDE_DIRS}
          PATH_SUFFIXES libad9361-iio)

find_library(LibAD9361_LIBRARY NAMES ad9361 libad9361
             HINTS ${PC_LibAD9361_LIBDIR} ${PC_LibAD9361_LIBRARY_DIRS})

set(LibAD9361_VERSION ${PC_LibAD9361_VERSION})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LibAD9361
                                  REQUIRED_VARS LibAD9361_LIBRARY LibAD9361_INCLUDE_DIR
                                  VERSION_VAR LibAD9361_VERSION)

mark_as_advanced(LibAD9361_INCLUDE_DIR LibAD9361_LIBRARY)

if (LibAD9361_FOUND)
set(LibAD9361_LIBRARIES ${LibAD9361_LIBRARY})
set(LibAD9361_INCLUDE_DIRS ${LibAD9361_INCLUDE_DIR})
endif()
