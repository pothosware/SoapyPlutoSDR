# - Try to find libad9361-iio
# Once done this will define
#
#  LIBAD9361_FOUND - system has libiio
#  LIBAD9361_INCLUDE_DIRS - the libiio include directory
#  LIBAD9361_LIBRARIES - Link these to use libiio
#  LIBAD9361_DEFINITIONS - Compiler switches required for using libiio
#
# Redistribution and use is allowed according to the terms of the New BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

include(FindPkgConfig)
pkg_check_modules(PC_LIBAD9361 QUIET libad9361)
set(LIBAD9361_DEFINITIONS ${PC_LIBAD9361_CFLAGS_OTHER})

find_path(LIBAD9361_INCLUDE_DIR ad9361.h
          HINTS ${PC_LIBAD9361_INCLUDEDIR} ${PC_LIBAD9361_INCLUDE_DIRS}
          PATH_SUFFIXES libad9361-iio)

find_library(LIBAD9361_LIBRARY NAMES ad9361 libad9361
             HINTS ${PC_LIBAD9361_LIBDIR} ${PC_LIBAD9361_LIBRARY_DIRS})

set(LIBAD9361_VERSION ${PC_LIBAD9361_VERSION})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libad9361
                                  REQUIRED_VARS LIBAD9361_LIBRARY LIBAD9361_INCLUDE_DIR
                                  VERSION_VAR LIBAD9361_VERSION)

mark_as_advanced(LIBAD9361_INCLUDE_DIR LIBAD9361_LIBRARY)

if (LIBAD9361_FOUND)
set(LIBAD9361_LIBRARIES ${LIBAD9361_LIBRARY})
set(LIBAD9361_INCLUDE_DIRS ${LIBAD9361_INCLUDE_DIR})
endif()