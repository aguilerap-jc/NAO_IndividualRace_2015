cmake_minimum_required(VERSION 2.8)
project(wlkv2)

find_package(qibuild)

# Create a executable named wlkv2
# with the source file: main.cpp
qi_create_bin(wlkv2 "main.cpp")

qi_use_lib(wlkv2 ALCOMMON ALPROXIES)
qi_use_lib(wlkv2 OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC OPENCV2_VIDEO)
