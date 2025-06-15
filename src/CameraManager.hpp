#pragma once

#include "CircularMatBuffer.hpp"
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <atomic>
#include <mutex>
#include <deque>
#include <memory>
#include <condition_variable>

class CameraManager {
public:
    CameraManager(size_t bufferSize = 30);
    ~CameraManager();

    void startCapture();
    cv::Mat getLatestFrame();
    std::deque<cv::Mat> getFrameBuffer();

private:
    void captureLoop();
    void handleRequest(libcamera::Request *request);
    cv::Mat convertFrame(libcamera::FrameBuffer *buffer, const libcamera::StreamConfiguration &config);

    std::thread captureThread;
    std::atomic<bool> running;
    // std::mutex bufferMutex;

    // std::deque<cv::Mat> frameBuffer;
    CircularMatBuffer frameBuffer;
    const size_t maxBufferSize = 150;

    std::shared_ptr<libcamera::CameraManager> camManager;
    std::shared_ptr<libcamera::Camera> camera;
    libcamera::StreamConfiguration currentStreamConfig;
    std::atomic<bool> shuttingDown = false;
    std::set<libcamera::Request *> inFlightRequests;
    std::mutex inFlightMutex;
    std::condition_variable inFlightCondition;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
};

