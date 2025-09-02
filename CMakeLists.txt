cmake_minimum_required(VERSION 3.10)

project(ChargeVideo VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fsanitize=address")
endif()

find_package(Corrade REQUIRED Main)
find_package(Magnum REQUIRED GL)
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)
pkg_check_modules(SWRESAMPLE REQUIRED libswresample)

add_library(ChargeVideo SHARED "src/ChargeVideo.hpp" "src/Time.cpp"
                               "src/Video.cpp")

target_link_libraries(
  ChargeVideo
  PRIVATE Corrade::Main
          Magnum::GL
          Magnum::Magnum
          ${AVFORMAT_LIBRARIES}
          ${AVCODEC_LIBRARIES}
          ${AVUTIL_LIBRARIES}
          ${SWSCALE_LIBRARIES}
          ${SWRESAMPLE_LIBRARIES})

target_include_directories(
  ChargeVideo PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
                     $<INSTALL_INTERFACE:include>)

# Library
install(
  TARGETS ChargeVideo
  EXPORT ChargeVideoTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# include
install(FILES src/ChargeVideo.hpp DESTINATION include/Charge)

install(
  EXPORT ChargeVideoTargets
  FILE ChargeVideoTargets.cmake
  NAMESPACE ChargeVideo::
  DESTINATION lib/cmake/ChargeVideo)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ChargeVideoConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/ChargeVideoConfig.cmake"
  INSTALL_DESTINATION lib/cmake/ChargeVideo)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ChargeVideoConfig.cmake"
        DESTINATION lib/cmake/ChargeVideo)
