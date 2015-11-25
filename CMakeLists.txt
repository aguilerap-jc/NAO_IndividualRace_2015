cmake_minimum_required(VERSION 2.8)
project(local_individual)

find_package(qibuild)

option(BUMPER_IS_REMOTE  "module is compiled as a remote module (ON or OFF)" ON)

# Create a executable named local_individual
# with the source file: main.cpp
#qi_create_bin(local_individual "bumper.h" "bumper.cpp" "main.cpp" "NaoVision.h" "NaoVision.cpp" "NaoMovement.h" "NaoMovement.cpp")

cmake_minimum_required(VERSION 2.8)
project(bumper)
find_package(qibuild)


if(BUMPER_IS_REMOTE)
  add_definitions(" -DBUMPER_IS_REMOTE ")
  qi_create_bin(local_individual "bumper.h" "bumper.cpp" "main.cpp" "NaoVision.h" "NaoVision.cpp" "NaoMovement.h" "NaoMovement.cpp")
else()
  qi_create_lib(local_individual SHARED "bumper.h" "bumper.cpp" "main.cpp" "NaoVision.h" "NaoVision.cpp" "NaoMovement.h" "NaoMovement.cpp" SUBFOLDER naoqi)
endif()

qi_use_lib(local_individual ALCOMMON ALPROXIES OPENCV2_CORE OPENCV2_IMGPROC OPENCV2_VIDEO OPENCV2_HIGHGUI)