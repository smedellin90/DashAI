cmake_minimum_required(VERSION 3.10)
project(DashAI)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
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
    # src/CircularMatBuffer.cpp
)

target_link_libraries(dashai
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    pthread
    -lcamera
    -lcamera-base
)

target_compile_options(dashai PRIVATE ${LIBCAMERA_CFLAGS_OTHER})
