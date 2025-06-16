#pragma once

#include <deque>
#include <mutex>
#include <vector>
#include <type_traits>
#include <opencv2/core.hpp>
#include <chrono>

template <typename T>
class CircularBuffer {
public:
    using Clock = std::chrono::steady_clock;
    using TimestampedEntry = std::pair<Clock::time_point, T>;

    CircularBuffer(size_t maxSize, std::chrono::milliseconds retentionDuration)
        : maxSize(maxSize), retention(retentionDuration) {}

    void push(const T& item) {
        std::lock_guard<std::mutex> lock(mutex);

        Clock::time_point now = Clock::now();

        // Purge stale frames based on retention duration
        while (!buffer.empty()) {
            auto frameTime = buffer.front().first;
            auto age = now - frameTime;

            if (age > retention) {
                buffer.pop_front();
            } else {
                break;  // Remaining frames are within retention
            }
        }
            // Enforce max size as well
            if (buffer.size() >= maxSize)
                buffer.pop_front();

            if constexpr (std::is_same<T, cv::Mat>::value)
                buffer.emplace_back(now, item.clone());
            else
                buffer.emplace_back(now, item);
    }

    T latest() const {
        std::lock_guard<std::mutex> lock(mutex);
        return buffer.empty() ? T() : buffer.back().second;
    }

    std::vector<T> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<T> result;
        for (const auto& [_, frame] : buffer)
            result.push_back(frame);
        return result;
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
    std::chrono::milliseconds retention;
    mutable std::mutex mutex;
    std::deque<TimestampedEntry> buffer;
};
