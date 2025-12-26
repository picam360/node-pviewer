cmake_minimum_required(VERSION 3.10)
project(imu-bmi088 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(HIREDIS REQUIRED hiredis)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${HIREDIS_INCLUDE_DIRS}
    ./include
)

target_link_libraries(${PROJECT_NAME}
    ${HIREDIS_LIBRARIES}
)
