if(TESTDATA_LOCATION)
    set(Deployment_Location ${TESTDATA_LOCATION})
else()
    #Windows OS will host the files under %TEMP% location
    #Unix-like machines will host the tests files under /tmp/ directory
    if (WIN32)
        set(Deployment_Location "$ENV{TEMP}\\")
    else() # Data shall be preserved between reboots. For Linux distributions/ ANDROID_NDK_TOOLCHAIN_INCLUDED/APPLE
        #set(Deployment_Location /var/tmp/) The standard configuration currently fails on CI
        set(Deployment_Location /tmp/)
    endif()
endif()


set(PP_Rosbag_Recordings_File D435i_Depth_and_IMU.bag)
set(PP_Rosbag_Recordings_URL https://librealsense.intel.com/rs-tests/Rosbag_unit_test_records)
dl_file( ${PP_Rosbag_Recordings_URL} ${} ${PP_Rosbag_Recordings_File} OFF )
