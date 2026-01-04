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
