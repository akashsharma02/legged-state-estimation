cmake_minimum_required(VERSION 3.15+)

# Project name
project(legged-state-estimation LANGUAGES CXX)
add_compile_options(-std=c++11)

# Project standard settings
# ------------------ Standard Project settings and warnings -----------------
include(cmake/standard_project_settings.cmake)
# Project options to be applicable for all the targets in the folder
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)
target_include_directories(project_options INTERFACE ${CMAKE_CURRENT_BINARY_DIR})

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# standard compiler warnings
include(cmake/compiler_warnings.cmake)
set_project_warnings(project_warnings)

# enable doxygen
include(cmake/doxygen.cmake)
enable_doxygen()


# ---------- Sanitizer and Static Analyzers to help for debugging -----------
# sanitizer options if supported by compiler
include(cmake/sanitizers.cmake)
enable_sanitizers(project_options)

# allow for static analysis options
include(cmake/static_analyzers.cmake)


# ----------------------- Dependencies --------------------------------------
include(cmake/conan.cmake)
run_conan()
find_package(Eigen3 3.3 REQUIRED)
find_package(GTSAM 4.1 REQUIRED)
find_package(yaml-cpp 0.6.0 PATHS thirdparty/yaml-cpp)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)
# ----------------------- Build options -------------------------------------

# ----------------------- Subdirectories ------------------------------------
add_subdirectory(utils)
add_subdirectory(src)


# add_subdirectory(scripts)

# ----------------------- Testing -------------------------------------------
