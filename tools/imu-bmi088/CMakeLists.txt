cmake_minimum_required(VERSION 3.10)
project(bmi088_redis C CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(HIREDIS REQUIRED hiredis)

add_executable(main main.cpp)

target_include_directories(main PRIVATE ${HIREDIS_INCLUDE_DIRS})
target_link_libraries(main ${HIREDIS_LIBRARIES})
