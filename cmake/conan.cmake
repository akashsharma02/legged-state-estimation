macro(run_conan)
  # Download automatically, you can also just copy the conan.cmake file
  if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(
      STATUS
      "Downloading conan.cmake from https://github.com/conan-io/cmake-conan"
    )
    file(
      DOWNLOAD "https://github.com/conan-io/cmake-conan/raw/v0.15/conan.cmake"
      "${CMAKE_BINARY_DIR}/conan.cmake"
      TLS_VERIFY ON
    )
  endif()

  include(${CMAKE_BINARY_DIR}/conan.cmake)

  conan_add_remote(
    NAME bincrafters URL
    https://api.bintray.com/conan/bincrafters/public-conan
  )

  conan_cmake_run(
    REQUIRES
    cli11/1.9.1
    fmt/7.1.2
    spdlog/1.8.2
    boost/1.75.0 #TODO: (Akash) Remove later after removing program options in run_dgs
    # gtsam/4.0.3
    OPTIONS
    ${CONAN_EXTRA_OPTIONS}
    BASIC_SETUP
    CMAKE_TARGETS # individual targets to link to
    BUILD
    missing
    INSTALL_FOLDER ${CMAKE_SOURCE_DIR}/.thirdparty/
    SETTINGS
    compiler.cppstd=14
  )
endmacro()
