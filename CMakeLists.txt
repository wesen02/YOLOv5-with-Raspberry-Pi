cmake_minimum_required(VERSION 3.0.2)
project(autopilot)

set(raspicam_DIR "/usr/local/include/raspicam")

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${raspicam_DIR}
)

add_executable(opencv src/main.cpp)
target_link_libraries(opencv ${OpenCV_LIBS} wiringPi raspicam_cv)
add_executable(capture src/capture.cpp)
target_link_libraries(capture ${OpenCV_LIBS} wiringPi raspicam_cv)