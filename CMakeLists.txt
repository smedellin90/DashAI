cmake_minimum_required(VERSION 3.10)
project(DashAI)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

# Use pkg-config to find libcamera flags
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
)

link_directories(${LIBCAMERA_LIBRARY_DIRS})

add_executable(dashai
    src/main.cpp
    src/CameraManager.cpp
)

target_link_libraries(dashai ${OpenCV_LIBS} ${LIBCAMERA_LIBS} pthread)
