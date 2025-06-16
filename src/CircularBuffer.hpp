#pragma once

#include <deque>
#include <mutex>
#include <vector>
#include <type_traits>
#include <opencv2/core.hpp> // Needed for cv::Mat

template <typename T>
class CircularBuffer {
public:
    CircularBuffer(size_t maxSize) : maxSize(maxSize) {}

    void push(const T& item) {
        std::lock_guard<std::mutex> lock(mutex);
        if (buffer.size() >= maxSize)
            buffer.pop_front();

        if constexpr (std::is_same<T, cv::Mat>::value) {
            buffer.push_back(item.clone()); // Deep copy for cv::Mat
        } else {
            buffer.push_back(item);
        }
    }

    T latest() const {
        std::lock_guard<std::mutex> lock(mutex);
        return buffer.empty() ? T() : buffer.back();
    }

    std::vector<T> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex);
        return std::vector<T>(buffer.begin(), buffer.end());
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex);
        return buffer.size();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex);
        buffer.clear();
    }

private:
    size_t maxSize;
    mutable std::mutex mutex;
    std::deque<T> buffer;
};
