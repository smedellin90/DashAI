#pragma once

#include <chrono>

struct CameraConfig {
    std::chrono::milliseconds preEventDuration;
    std::chrono::milliseconds postEventDuration;
    size_t bufferCapacity;
};