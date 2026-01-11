cmake_minimum_required(VERSION 3.16)
project(pserver_vio CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)

find_path(HIREDIS_INCLUDE_DIR hiredis/hiredis.h)
find_library(HIREDIS_LIBRARY hiredis)

if(NOT HIREDIS_INCLUDE_DIR OR NOT HIREDIS_LIBRARY)
  message(FATAL_ERROR "hiredis not found. sudo apt-get install libhiredis-dev")
endif()

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${HIREDIS_INCLUDE_DIR}
  ./include
  /home/picam360/github/basalt/include
  /home/picam360/github/basalt/thirdparty/basalt-headers/include
  /home/picam360/github/basalt/thirdparty/basalt-headers/thirdparty/eigen
  /home/picam360/github/basalt/thirdparty/basalt-headers/thirdparty/cereal/include
  /home/picam360/github/basalt/thirdparty/basalt-headers/thirdparty/Sophus
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${OpenCV_LIBS}
  ${HIREDIS_LIBRARY}
  TBB::tbb
  /home/picam360/github/basalt/build/libbasalt.so
)

# ============================================================
# Precompiled headers (PCH)
# ============================================================
# CMake 3.16 supports target_precompile_headers()
# Good candidates: heavy STL / Eigen / OpenCV / json
target_precompile_headers(${PROJECT_NAME} PRIVATE
  <atomic>
  <thread>
  <mutex>
  <condition_variable>
  <deque>
  <vector>
  <string>
  <iostream>
  <fstream>
  <regex>

  <Eigen/Core>
  <Eigen/Geometry>

  <opencv2/opencv.hpp>

  <nlohmann/json.hpp>

  <tbb/concurrent_queue.h>

  <hiredis/hiredis.h>

  <basalt/vi_estimator/vio_estimator.h>
  <basalt/calibration/calibration.hpp>
  <basalt/utils/time_utils.hpp>
  <basalt/serialization/headers_serialization.h>
)

# (Optional) make PCH more stable across builds
# If you often change compile options, PCH can rebuild frequently.
# set_property(TARGET ${PROJECT_NAME} PROPERTY PCH_WARN_INVALID OFF)
