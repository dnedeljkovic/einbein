add_executable(digIO digIO.cpp)
target_link_libraries(digIO eeros flink)

add_executable(fqdMot fqdMot.cpp)
target_link_libraries(fqdMot eeros flink)

add_executable(dacMot dacMot.cpp)
target_link_libraries(dacMot eeros flink)

add_executable(imu imu.cpp)
target_link_libraries(imu mpu9250 eeros)