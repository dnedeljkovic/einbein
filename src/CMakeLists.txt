add_subdirectory(mpu9250)

add_executable(einbein main.cpp)
target_link_libraries(einbein eeros flink)
