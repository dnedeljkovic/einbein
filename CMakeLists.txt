cmake_minimum_required(VERSION 2.8)

project(einbein)

include_directories(/home/dejan/eeros-framework/includes)
include_directories(/home/dejan/flinklib/include)
link_directories(/home/dejan/eeros-framework/build-arm/src)
link_directories(/home/dejan/flinklib/build-arm/lib)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_subdirectory(src)
add_subdirectory(test)