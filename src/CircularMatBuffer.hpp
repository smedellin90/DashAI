#pragma once

#include <opencv2/core.hpp>
#include <deque>
#include <mutex>

class CircularMatBuffer {
public:
    explicit CircularMatBuffer(size_t capacity);

    void push(const cv::Mat &frame);
    cv::Mat latest() const;
    std::deque<cv::Mat> snapshot() const;
    size_t size() const;

private:
    size_t capacity_;
    std::deque<cv::Mat> buffer_;
    mutable std::mutex mutex_;
};
