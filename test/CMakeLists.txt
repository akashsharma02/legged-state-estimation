add_executable(
  oneLeg
  one-leg.cpp
)

target_link_libraries(
  oneLeg
  PRIVATE
  ${PROJECT_NAME}
)

add_executable(
  inertial
  inertialOnly.cpp
)

target_link_libraries(
  inertial
  PRIVATE
  project_options
  project_warnings
  utils
  CONAN_PKG::fmt
  CONAN_PKG::spdlog
  CONAN_PKG::cli11
  gtsam
  ${OpenCV_LIBS}
  ${PROJECT_NAME}
)
