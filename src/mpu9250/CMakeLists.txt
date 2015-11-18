add_library(mpu9250
	    eMPL/inv_mpu.c 
	    eMPL/inv_mpu_dmp_motion_driver.c
	    glue/linux_glue.c
	    mpu9250/mpu9250.c
	    mpu9250/quaternion.c
	    mpu9250/vector3d.c
)

#target_link_libraries(mpu9250 eeros) # Link mpu9250 library against libeeros