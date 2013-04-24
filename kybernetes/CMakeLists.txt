# CMakeLists.txt to set up the compilation of the software that drives
# Kybernetes, our navigations platform
cmake_minimum_required (VERSION 2.6) 
project (KYBERNETES) 

# Setup project directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)

# Use the NEON fpu
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpu=neon")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -std=c++0x")

# Tell CMAKE to invoke gcc on assembly sources
set_property(SOURCE src/kybernetes/cv/yuv422_bithreshold.s PROPERTY LANGUAGE C)

# Include path
include_directories (${KYBERNETES_SOURCE_DIR}/include) 

# Create the drivers library for Kybernetes, all applications will link
# with this shared library
add_library(kybernetes SHARED src/kybernetes/controller/motion_controller.cpp
                              src/kybernetes/controller/sensor_controller.cpp
                              src/kybernetes/io/serial.cpp
                              src/kybernetes/network/serversocket.cpp
                              src/kybernetes/network/socket.cpp
                              src/kybernetes/sensor/garmingps.cpp
                              src/kybernetes/sensor/nmeagps.cpp
                              src/kybernetes/sensor/razorimu.cpp
                              src/kybernetes/sensor/razorgyro.cpp
                              src/kybernetes/sensor/uvccamera.cpp
                              src/kybernetes/math/gps_common.cpp
                              src/kybernetes/cv/yuv422_bithreshold.s
           )

# Link our library to boost
target_link_libraries (kybernetes boost_thread-mt)
target_link_libraries (kybernetes boost_date_time-mt)
target_link_libraries (kybernetes boost_system-mt)

# Build the obstacle avoidance application
add_executable(avoid_demo src/demos/avoid.cpp)
target_link_libraries(avoid_demo kybernetes)

# Build the heading follow application
add_executable(imu_hold_demo src/demos/imu_hold.cpp)
target_link_libraries(imu_hold_demo kybernetes)

# Build the GPS goto application
add_executable(gps_navigate_demo src/demos/gps_navigate.cpp)
target_link_libraries(gps_navigate_demo kybernetes)

# Build the Blob tracking daemon
add_executable(blobtrackd src/blobtrack/blobtrackd.cpp)
target_link_libraries(blobtrackd kybernetes)
target_link_libraries(blobtrackd boost_thread-mt)
target_link_libraries(blobtrackd boost_date_time-mt)
target_link_libraries(blobtrackd boost_system-mt)
target_link_libraries(blobtrackd opencv_core)
target_link_libraries(blobtrackd opencv_imgproc)
target_link_libraries(blobtrackd cvblobs)
target_link_libraries(blobtrackd jsoncpp)

# Build the Blob tracker applications
#add_executable(blob_track_demo src/applications/blob_track_demo.cpp)
#target_link_libraries(blob_track_demo kybernetes)
#target_link_libraries(blob_track_demo boost_thread-mt)
#target_link_libraries(blob_track_demo boost_date_time-mt)
#target_link_libraries(blob_track_demo boost_system-mt)
#target_link_libraries(blob_track_demo opencv_core)
#target_link_libraries(blob_track_demo opencv_imgproc)
#target_link_libraries(blob_track_demo cvblobs)


