set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(
    CMAKE_BUILD_TYPE
    RelWithDebInfo
    CACHE STRING "Choose the type of build." FORCE
  )
  # Set the possible values of build type for cmake-gui, ccmake
  set_property(
    CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo"
  )
endif()

find_program(CCACHE ccache)
if(CCACHE)
  message("using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE})
else()
  message("ccache not found cannot use")
endif()

# Generate compile_commands.json to make it easier to work with clang based
# tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(
  ENABLE_IPO
  "Enable Iterprocedural Optimization, aka Link Time Optimization (LTO)"
  OFF
)

if(ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT result OUTPUT output)
  if(result)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(SEND_ERROR "IPO is not supported: ${output}")
  endif()
endif()

# Allow to set the debug level from cmake directly
set(
  LOGGER_ACTIVE_LEVEL
  LEVEL_DEBUG
  CACHE STRING "Set the logger level"
)

set_property(
  CACHE LOGGER_ACTIVE_LEVEL PROPERTY STRINGS
  "LEVEL_TRACE"
  "LEVEL_DEBUG"
  "LEVEL_INFO"
  "LEVEL_WARN"
  "LEVEL_ERROR"
  "LEVEL_CRITICAL"
  "LEVEL_OFF"
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/logging.h.in logging.h @ONLY)
