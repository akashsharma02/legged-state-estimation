##################### Add subdirectories ##########################
add_executable(
  test
  test.cpp
)

target_link_libraries(
  test
  PRIVATE
  project_options
  project_warnings
  utils
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  CONAN_PKG::cli11
  gtsam
)
