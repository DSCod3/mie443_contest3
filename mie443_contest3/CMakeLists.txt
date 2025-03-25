cmake_minimum_required(VERSION 3.0.2)
project(mie443_contest3)

# Set global C++14 standard first
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Find required packages
find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    cv_bridge 
    image_transport 
    sensor_msgs
    roscpp
    std_msgs
)

# Find PortAudio
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

# Include directories
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PORTAUDIO_INCLUDE_DIRS}
)

# Link directories
link_directories(${PORTAUDIO_LIBRARY_DIRS})

# Catkin package setup
catkin_package()

# Sound direction node
add_executable(sound_direction
    src/sound_direction.cpp
)
target_compile_options(sound_direction PRIVATE
    -O3
    -march=native
    -ffast-math
    -flto
)
target_link_libraries(sound_direction
    ${catkin_LIBRARIES}
    ${PORTAUDIO_LIBRARIES}
    pthread
)
set_target_properties(sound_direction PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

# Publisher node
add_executable(publisher src/my_publisher_video.cpp)
target_link_libraries(publisher
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
set_target_properties(publisher PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

# Contest3 node
add_executable(contest3 
    src/contest3.cpp
    src/imageTransporter.cpp 
    src/bumper.cpp 
    src/cliff.cpp
)
target_link_libraries(contest3
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
set_target_properties(contest3 PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)