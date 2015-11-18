cmake_minimum_required(VERSION 2.8)

project(einbein)

## Additional include and link directories
include_directories(/home/dejan/eeros-framework/includes /home/dejan/flinklib/include)
link_directories(/home/dejan/eeros-framework/build-arm/src /home/dejan/flinklib/build-arm/lib "${CMAKE_BINARY_DIR}/src/mpu9250")


set(CMAKE_CXX_FLAGS "-std=c++11")

add_subdirectory(src)
add_subdirectory(test)