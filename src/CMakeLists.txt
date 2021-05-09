##################### Add subdirectories ##########################

add_library(${PROJECT_NAME} SHARED
    dataloader.cpp
    # imu.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    project_options
    project_warnings
    utils
    Eigen3::Eigen
    CONAN_PKG::fmt
    CONAN_PKG::cli11
    CONAN_PKG::spdlog
    CONAN_PKG::yaml-cpp
    CONAN_PKG::fast-cpp-csv-parser
    gtsam
)

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
  one-leg.cpp
)

target_link_libraries(
  oneLeg
  PRIVATE
  ${PROJECT_NAME}
)

add_executable(
  testTwist
  testTwist.cpp
)

target_link_libraries(
  testTwist
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

add_executable(
  imuToy
  imufactorToyExample.cpp
)

target_link_libraries(
  imuToy
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

