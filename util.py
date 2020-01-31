from math import *
import numpy as np

# the number of degrees per meter of circumference on Earth (radius of 6.371 million meters)
# this is a suitable approximation as the low altitude of the UAV will produce negligible differences in latitude and longitude
DEGREES_PER_METER = 360 / (2 * 6.371e6 * pi)

# generate a rotation matrix around the x-axis
def matrix_rot_x(theta):
    s, c = sin(theta), cos(theta)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])

# generate a rotation matrix around the y-axis
def matrix_rot_y(theta):
    s, c = sin(theta), cos(theta)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])

# generate a rotation matrix around the z-axis
def matrix_rot_z(theta):
    s, c = sin(theta), cos(theta)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])

# generate a gstreamer pipeline string for video capture
def gstreamer_pipeline(capture_size=(1280, 720), display_size=(1280, 720), framerate=60, flip_method=0):
    return (
        'nvarguscamerasrc exposuretimerange="10000000 10000000" tnr-strength=1 tnr-mode=2 ee-strength=0.5 aeantibanding=0 maxperf=true ! '
        'video/x-raw(memory:NVMM), '
        'width=%d, height=%d, '
        'format=NV12, framerate=%d/1 ! '
        'nvvidconv flip-method=%d ! '
        'video/x-raw, width=%d, height=%d, format=BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=BGR ! appsink'
        % (*capture_size, framerate, flip_method, *display_size)
    )