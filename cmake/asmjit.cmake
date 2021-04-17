#======================================================================================
# asmjit.cmake
#
# Configure asmjit for Dyninst
#
#   ----------------------------------------
#
# Accepts the following CMake variables
#
# asmjit_ROOT_DIR            - Base directory of the asmjit installation
# asmjit_INCLUDEDIR          - Hint directory that contains the asmjit headers files
# asmjit_LIBRARYDIR          - Hint directory that contains the asmjit library files
# asmjit_MIN_VERSION         - Minimum acceptable version of asmjit
# asmjit_USE_STATIC_RUNTIME  - Link to the static runtime library
#
# Exports the following CMake cache variables
#
# asmjit_ROOT_DIR       - Base directory of the asmjit installation
# asmjit_INCLUDE_DIRS   - asmjit include directories
# asmjit_LIBRARY_DIRS   - Link directories for asmjit libraries
# asmjit_LIBRARIES      - asmjit library files
#
#======================================================================================

if(asmjit_FOUND)
  return()
endif()

# -------------- VERSION ------------------------------------------------------

# Minimum acceptable version
#set(_min_version 8.0)

#set(asmjit_MIN_VERSION ${_min_version}
#    CACHE STRING "Minimum acceptable capstone version")

#if(${asmjit_MIN_VERSION} VERSION_LESS ${_min_version})
#  message(
#    FATAL_ERROR
#      "Requested version ${asmjit_MIN_VERSION} is less than minimum supported version (${_min_version})"
#    )
#endif()

# -------------- PATHS --------------------------------------------------------

# Base directory the of capstone installation
set(asmjit_ROOT_DIR "/usr"
    CACHE PATH "Base directory the of capstone installation")

# Hint directory that contains the capstone headers files
set(asmjit_INCLUDEDIR "${asmjit_ROOT_DIR}/include"
    CACHE PATH "Hint directory that contains the capstone headers files")

# Hint directory that contains the capstone library files
set(asmjit_LIBRARYDIR "${asmjit_ROOT_DIR}/lib64"
    CACHE PATH "Hint directory that contains the capstone library files")

# -----------------------------------------------------------------------------

find_package(asmjit ${asmjit_MIN_VERSION} REQUIRED)

if(NOT asmjit_FOUND)
  message(
    FATAL_ERROR
      "asmjit was not found. See https://github.com/dyninst/dyninst/wiki/third-party-deps#capstone for instructions on building it from source."
    )
endif()

link_directories(${asmjit_LIBRARY_DIRS})
include_directories(${asmjit_INCLUDE_DIRS})

message(STATUS "asmjit includes: ${asmjit_INCLUDE_DIRS}")
message(STATUS "asmjit library dirs: ${asmjit_LIBRARY_DIRS}")
message(STATUS "asmjit libraries: ${asmjit_LIBRARIES}")

if(USE_COTIRE)
  cotire(asmjit)
endif()
