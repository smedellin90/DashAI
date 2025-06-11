#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP

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

class CameraManager {
public:
    CameraManager();
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
    std::mutex bufferMutex;

    std::deque<cv::Mat> frameBuffer;
    const size_t maxBufferSize = 150;

    std::shared_ptr<libcamera::CameraManager> camManager;
    std::shared_ptr<libcamera::Camera> camera;
    libcamera::StreamConfiguration currentStreamConfig;
};

#endif // CAMERA_MANAGER_HPP
