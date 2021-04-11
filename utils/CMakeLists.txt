# Use project options to automatically include the utils to all the executables or libraries
add_library(utils)

target_sources(
  utils
  PUBLIC
  utils.h
  utils.cpp
)

target_include_directories(
  utils
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}
)



target_link_libraries(
  utils
  PRIVATE
  project_options
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  gtsam
)
