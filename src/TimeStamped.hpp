#pragma once

#include <opencv2/core.hpp>
#include <chrono>

template<typename T>
struct Timestamped {
    T frame;
    std::chrono::steady_clock::time_point timestamp;
};