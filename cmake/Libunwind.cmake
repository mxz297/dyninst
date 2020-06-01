#======================================================================================
# Libunwind.cmake
#
# Configure Libunwind for Dyninst
#
#   ----------------------------------------
#
# Accepts the following CMake variables
#
# Libunwind_ROOT_DIR            - Base directory of the Libunwind installation
# Libunwind_INCLUDEDIR          - Hint directory that contains the Libunwind headers files
# Libunwind_LIBRARYDIR          - Hint directory that contains the Libunwind library files
# Libunwind_MIN_VERSION         - Minimum acceptable version of Libunwind
# Libunwind_USE_STATIC_RUNTIME  - Link to the static runtime library
#
# Exports the following CMake cache variables
#
# Libunwind_ROOT_DIR       - Base directory of the Libunwind installation
# Libunwind_INCLUDE_DIRS   - Libunwind include directories
# Libunwind_LIBRARY_DIRS   - Link directories for Libunwind libraries
# Libunwind_LIBRARIES      - Libunwind library files
#
#======================================================================================

if(Libunwind_FOUND)
  return()
endif()

# -------------- VERSION ------------------------------------------------------

# Minimum acceptable version
#set(_min_version 8.0)

#set(Libunwind_MIN_VERSION ${_min_version}
#    CACHE STRING "Minimum acceptable capstone version")

#if(${Libunwind_MIN_VERSION} VERSION_LESS ${_min_version})
#  message(
#    FATAL_ERROR
#      "Requested version ${Libunwind_MIN_VERSION} is less than minimum supported version (${_min_version})"
#    )
#endif()

# -------------- PATHS --------------------------------------------------------

# Base directory the of capstone installation
set(Libunwind_ROOT_DIR "/usr"
    CACHE PATH "Base directory the of capstone installation")

# Hint directory that contains the capstone headers files
set(Libunwind_INCLUDEDIR "${Libunwind_ROOT_DIR}/include"
    CACHE PATH "Hint directory that contains the capstone headers files")

# Hint directory that contains the capstone library files
set(Libunwind_LIBRARYDIR "${Libunwind_ROOT_DIR}/lib64"
    CACHE PATH "Hint directory that contains the capstone library files")

# -----------------------------------------------------------------------------

find_package(Libunwind ${Libunwind_MIN_VERSION} REQUIRED)

if(NOT Libunwind_FOUND)
  message(
    FATAL_ERROR
      "Libunwind was not found. See https://github.com/dyninst/dyninst/wiki/third-party-deps#capstone for instructions on building it from source."
    )
endif()

link_directories(${Libunwind_LIBRARY_DIRS})
include_directories(${Libunwind_INCLUDE_DIRS})

message(STATUS "Libunwind includes: ${Libunwind_INCLUDE_DIRS}")
message(STATUS "Libunwind library dirs: ${Libunwind_LIBRARY_DIRS}")
message(STATUS "Libunwind libraries: ${Libunwind_LIBRARIES}")

if(USE_COTIRE)
  cotire(Libunwind)
endif()
