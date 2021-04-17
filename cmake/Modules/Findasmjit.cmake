#===================================================================================
# Findasmjit.cmake
#
# Find asmjit include dirs and libraries
#
#		----------------------------------------
#
# Use this module by invoking find_package with the form
#
#  find_package(asmjit
#    [version] [EXACT]      # Minimum or EXACT version e.g. 5.0
#    [REQUIRED]             # Fail with error if asmjit is not found
#  )
#
# This module reads hints about search locations from variables::
#
#	asmjit_ROOT_DIR		     - Base directory the of asmjit installation
#	asmjit_INCLUDEDIR		     - Hint directory that contains the asmjit headers files
#	asmjit_LIBRARYDIR		     - Hint directory that contains the asmjit library files
#   asmjit_USE_STATIC_RUNTIME  - Use static runtime
#
# and saves search results persistently in CMake cache entries::
#
#	asmjit_FOUND			- True if headers and requested libraries were found
#	asmjit_INCLUDE_DIRS 	- asmjit include directories
#	asmjit_LIBRARY_DIRS	- Link directories for asmjit libraries
#	asmjit_LIBRARIES		- asmjit library files
#
#===================================================================================

include(DyninstSystemPaths)

find_path(asmjit_INCLUDE_DIR
          NAMES asmjit/asmjit.h
          HINTS ${asmjit_ROOT_DIR}/include ${asmjit_ROOT_DIR}
                ${asmjit_INCLUDEDIR}
          PATHS ${DYNINST_SYSTEM_INCLUDE_PATHS}
          PATH_SUFFIXES asmjit
          DOC "asmjit include directories")

find_library(asmjit_LIBRARIES
             NAMES "libasmjit.a"
             HINTS ${asmjit_ROOT_DIR} ${asmjit_LIBRARYDIR}
                   ${asmjit_ROOT_DIR}/lib ${asmjit_ROOT_DIR}/lib64
             PATHS ${DYNINST_SYSTEM_LIBRARY_PATHS}
             PATH_SUFFIXES asmjit)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(asmjit
                                  FOUND_VAR
                                  asmjit_FOUND
                                  REQUIRED_VARS
                                  asmjit_LIBRARIES
                                  asmjit_INCLUDE_DIR
                                  VERSION_VAR
                                  asmjit_VERSION)

# Export cache variables
if(asmjit_FOUND)
  set(asmjit_INCLUDE_DIRS
      ${asmjit_INCLUDE_DIR} 		# for #include "asmjit/asmjit.h"
      CACHE PATH "asmjit include directories")
  get_filename_component(_asmjit_lib_dir ${asmjit_LIBRARIES} DIRECTORY)
  set(asmjit_LIBRARY_DIRS ${_asmjit_lib_dir}
      CACHE PATH "Link directories for asmjit libraries")

  add_library(asmjit STATIC IMPORTED)
endif()

unset(_asmjit_lib_dir)
