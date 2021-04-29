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

add_executable(
  oneLeg
  oneLegTester.cpp
)

target_link_libraries(
  oneLeg
  PRIVATE
  project_options
  project_warnings
  utils
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  CONAN_PKG::cli11
  gtsam
  yaml-cpp
)


add_executable(
  yamltester
  yamltester.cpp
)

target_link_libraries(
  yamltester
  PRIVATE
  project_options
  project_warnings
  utils
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  CONAN_PKG::cli11
  yaml-cpp
  gtsam
  ${EIGEN3_INCLUDE_DIR}
)