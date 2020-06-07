#===================================================================================
# FindLibunwind.cmake
#
# Find Libunwind include dirs and libraries
#
#		----------------------------------------
#
# Use this module by invoking find_package with the form
#
#  find_package(Libunwind
#    [version] [EXACT]      # Minimum or EXACT version e.g. 5.0
#    [REQUIRED]             # Fail with error if Libunwind is not found
#  )
#
# This module reads hints about search locations from variables::
#
#	Libunwind_ROOT_DIR		     - Base directory the of capstone installation
#	Libunwind_INCLUDEDIR		     - Hint directory that contains the capstone headers files
#	Libunwind_LIBRARYDIR		     - Hint directory that contains the capstone library files
#   Libunwind_USE_STATIC_RUNTIME  - Use static runtime
#
# and saves search results persistently in CMake cache entries::
#
#	Libunwind_FOUND			- True if headers and requested libraries were found
#	Libunwind_INCLUDE_DIRS 	- capstone include directories
#	Libunwind_LIBRARY_DIRS	- Link directories for capstone libraries
#	Libunwind_LIBRARIES		- capstone library files
#
#===================================================================================

include(DyninstSystemPaths)

if(Libunwind_USE_STATIC_RUNTIME)
  set(_libunwind_lib_name "libunwind.a")
endif()

find_path(Libunwind_INCLUDE_DIR
          NAMES libunwind.h
          HINTS ${Libunwind_ROOT_DIR}/include ${Libunwind_ROOT_DIR}
                ${Libunwind_INCLUDEDIR}
          PATHS ${DYNINST_SYSTEM_INCLUDE_PATHS}
          PATH_SUFFIXES capstone
          DOC "capstone include directories")

find_library(Libunwind_LIBRARIES
             NAMES "libunwind.so"
             HINTS ${Libunwind_ROOT_DIR} ${Libunwind_LIBRARYDIR}
                   ${Libunwind_ROOT_DIR}/lib ${Libunwind_ROOT_DIR}/lib64
             PATHS ${DYNINST_SYSTEM_LIBRARY_PATHS}
             PATH_SUFFIXES libunwind)

if (PLATFORM MATCHES amd64 OR PLATFORM MATCHES x86_64)
set(_libunwind_lib_name "libunwind-x86_64.so")
elseif (PLATFORM MATCHES ppc64)
set(_libunwind_lib_name "libunwind-ppc64.so")
elseif (PLATFORM MATCHES aarch64)
set(_libunwind_lib_name "libunwind-aarch64.so")
endif()

find_library(Libunwind_plat_LIBRARIES
             NAMES ${_libunwind_lib_name}
             HINTS ${Libunwind_ROOT_DIR} ${Libunwind_LIBRARYDIR}
                   ${Libunwind_ROOT_DIR}/lib ${Libunwind_ROOT_DIR}/lib64
             PATHS ${DYNINST_SYSTEM_LIBRARY_PATHS}
             PATH_SUFFIXES libunwind)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Libunwind
                                  FOUND_VAR
                                  Libunwind_FOUND
                                  REQUIRED_VARS
                                  Libunwind_LIBRARIES
                                  Libunwind_INCLUDE_DIR
                                  VERSION_VAR
                                  Libunwind_VERSION)

# Export cache variables
if(Libunwind_FOUND)
  set(Libunwind_INCLUDE_DIRS
      ${Libunwind_INCLUDE_DIR} 		# for #include "capstone.h"
      ${Libunwind_INCLUDE_DIR}/.. 	# for #include "capstone/capstone.h"
      CACHE PATH "Libunwind include directories")
  get_filename_component(_libunwind_dir ${Libunwind_LIBRARIES} DIRECTORY)
  set(Libunwind_LIBRARY_DIRS ${_libunwind_dir}
      CACHE PATH "Link directories for capstone libraries")
  set(Libunwind_LIBRARIES ${Libunwind_LIBRARIES} ${Libunwind_plat_LIBRARIES})

  if(Libunwind_USE_STATIC_RUNTIME)
    add_library(libunwind STATIC IMPORTED)
  else()
    add_library(libunwind SHARED IMPORTED)
  endif()
endif()

unset(_libunwind_lib_name)
