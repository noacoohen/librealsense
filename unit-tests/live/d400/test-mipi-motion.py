# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2024 Intel Corporation. All Rights Reserved.

# test:device:jetson D457

import pyrealsense2 as rs
from rspy import test
import time
gyro_frame_count = 0
accel_frame_count = 0

def frame_callback( f ):
    global gyro_frame_count,accel_frame_count
    stream_type = f.get_profile().stream_type()
    if stream_type == rs.stream.gyro:
        gyro_frame_count += 1
        test.check_equal(f.get_frame_number(),gyro_frame_count)
    elif stream_type == rs.stream.accel:
        accel_frame_count += 1
        test.check_equal(f.get_frame_number(), accel_frame_count)

################################################################################################
with test.closure("frame index - mipi IMU "):
    seconds_to_count_frames = 10
    dev = test.find_first_device_or_exit()
    sensor = dev.first_motion_sensor()
    sensor.open( sensor.get_stream_profiles()[0] )
    sensor.start( frame_callback )
    time.sleep(seconds_to_count_frames) # Time to count frames
    sensor.stop()
    sensor.close()