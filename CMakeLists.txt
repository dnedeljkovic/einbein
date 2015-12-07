cmake_minimum_required(VERSION 2.8)

project(einbein)

## Additional include and link directories for flink and eeros
include_directories(/home/dejan/eeros-framework/includes /home/dejan/flinklib/include)
link_directories(/home/dejan/eeros-framework/build-arm/src /home/dejan/flinklib/build-arm/lib "${CMAKE_BINARY_DIR}/src/mpu9250")
include_directories(/home/reto/eeros-framework/includes /home/reto/flinklib/include)
link_directories(/home/reto/eeros-framework/build-arm/src /home/reto/flinklib/build-arm/lib "${CMAKE_BINARY_DIR}/src/mpu9250")


set(CMAKE_CXX_FLAGS "-std=c++11")

## Additional include and link directories
include_directories(include)

add_subdirectory(src)
add_subdirectory(test)

<<<<<<< HEAD

=======
>>>>>>> 9f96e30b121cb26268f32b33fc2e8f1c5b32c676
