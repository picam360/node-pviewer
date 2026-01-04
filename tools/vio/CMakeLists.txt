cmake_minimum_required(VERSION 3.16)
project(basalt_redis_vio CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

find_path(HIREDIS_INCLUDE_DIR hiredis/hiredis.h)
find_library(HIREDIS_LIBRARY hiredis)

if(NOT HIREDIS_INCLUDE_DIR OR NOT HIREDIS_LIBRARY)
  message(FATAL_ERROR "hiredis not found. sudo apt-get install libhiredis-dev")
endif()

add_executable(basalt_redis_vio basalt_redis_vio.cpp)

target_include_directories(basalt_redis_vio PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${HIREDIS_INCLUDE_DIR}
  ./include
)

target_link_libraries(basalt_redis_vio PRIVATE
  ${OpenCV_LIBS}
  ${HIREDIS_LIBRARY}
)

# Basalt をリンクする場合（あなたの環境に合わせて設定）
# 例:
# target_include_directories(basalt_redis_vio PRIVATE /path/to/basalt/include /path/to/basalt-headers/include)
# target_link_directories(basalt_redis_vio PRIVATE /path/to/basalt/build/lib)
# target_link_libraries(basalt_redis_vio PRIVATE basalt)
