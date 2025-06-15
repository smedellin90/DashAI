#include "CircularMatBuffer.hpp"

CircularMatBuffer::CircularMatBuffer(size_t capacity) : capacity_(capacity) {}

void CircularMatBuffer::push(const cv::Mat &frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.size() >= capacity_)
        buffer_.pop_front();
    buffer_.push_back(frame.clone()); // clone to ensure we own the memory
}

cv::Mat CircularMatBuffer::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.empty() ? cv::Mat() : buffer_.back();
}

std::deque<cv::Mat> CircularMatBuffer::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_;
}

size_t CircularMatBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
}
