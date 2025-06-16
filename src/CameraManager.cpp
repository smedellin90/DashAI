#include "CameraManager.hpp"
#include "libcamera/camera.h"

#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/stream.h>
#include <libcamera/request.h>
#include <libcamera/formats.h>

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <functional>

// using namespace libcamera;

CameraManager::CameraManager(size_t bufferSize) 
    : running(false),
    frameBuffer(bufferSize) {}

CameraManager::~CameraManager() {
    running = false;
    shuttingDown = true;

    if (captureThread.joinable())
        captureThread.join();

    {
        std::unique_lock<std::mutex> lock(inFlightMutex);
        inFlightCondition.wait(lock, [this]() {
            return inFlightRequests.empty();
        });
    }

    if (camera) {
        camera->stop();

        if (allocator && currentStreamConfig.stream()) {
            if (allocator->free(currentStreamConfig.stream()) < 0) {
                std::cerr << "[WARN] Failed to free buffers." << std::endl;
            }
        }
        allocator.reset();

        camera->release();
        camera.reset();
    }

    if (camManager)
        camManager->stop();
}

void CameraManager::startCapture() {
    running = true;
    captureThread = std::thread(&CameraManager::captureLoop, this);
}

void CameraManager::captureLoop() {
    std::cout << "[INFO] Starting Capture loop." << std::endl;

    camManager = std::make_shared<libcamera::CameraManager>();
    camManager->start();

    if (camManager->cameras().empty()) 
    {
        std::cerr << "No cameras available." << std::endl;
        return;
    }

    camera = camManager->get(camManager->cameras()[0]->id());
    if (!camera || camera->acquire()) 
    {
        std::cerr << "Failed to acquire camera." << std::endl;
        return;
    }

    std::cout << "[INFO] Creating roles." << std::endl; 

    std::vector<libcamera::StreamRole> roles = { libcamera::StreamRole::Viewfinder };
    std::unique_ptr<libcamera::CameraConfiguration> config = camera->generateConfiguration(roles);
    if (!config || config->size() != 1) 
    {
        std::cerr << "Failed to generate configuration." << std::endl;
        return;
    }

    std::cout << "[INFO] Setting up config." << std::endl;  

    config->at(0).pixelFormat = libcamera::formats::RGB888;
    config->at(0).size = {640, 480};
    if (config->validate() == libcamera::CameraConfiguration::Invalid)
    {
        std::cerr << "Camera configuration is invalid." << std::endl;
        return;
    }

    if (camera->configure(config.get()) < 0) 
    {
        std::cerr << "Camera configuration failed." << std::endl;
        return;
    }

    currentStreamConfig = config->at(0);

    libcamera::Stream *stream = currentStreamConfig.stream();

    allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera);
    if (allocator->allocate(stream) < 0)
    {
        std::cerr << "Buffer allocation failed." << std::endl;
        return;
    }

    const auto &buffers = allocator->buffers(stream);
    if (buffers.empty()) 
    {
        std::cerr << "No buffers available." << std::endl;
        return;
    }

    if (camera->start() < 0) 
    {
        std::cerr << "Camera start failed." << std::endl;
        return;
    }

    camera->requestCompleted.connect(this, [this](libcamera::Request *request) { 
        handleRequest(request);
    });

    for (const auto &buffer : buffers) {
        std::unique_ptr<libcamera::Request> request = camera->createRequest();
        if (!request || request->addBuffer(stream, buffer.get()) < 0) {
            std::cerr << "Request creation or buffer binding failed." << std::endl;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(inFlightMutex);
            inFlightRequests.insert(request.get());
        }

        camera->queueRequest(request.release());
    }

    while (running) 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void CameraManager::handleRequest(libcamera::Request *request) {
    if (request->status() == libcamera::Request::RequestCancelled)
        return;

    for (auto &[_ , buffer] : request->buffers()) {
        cv::Mat frame = convertFrame(buffer, currentStreamConfig);

        if (!frame.empty()) {
            frameBuffer.push(frame);
            if (!eventTriggered && detectEvent(frame)) {
                std::cout << "[EVENT] Motion detected. Saving buffer..." << std::endl;
                eventTriggered = true;

                // Save pre-event frames
                std::vector<cv::Mat> snapshot = frameBuffer.snapshot();
                int index = 0;
                for (const auto &f : snapshot) {
                    std::string filename = "event_frame_pre_" + std::to_string(index++) + ".jpg";
                    cv::imwrite(filename, f);
                }
                postEventFrameCount = 0;
                postEventFrames.clear();
            }

            if (eventTriggered) {
                postEventFrames.push_back(frame);
                postEventFrameCount++;

                if (postEventFrameCount >= postEventFramesToCapture) {
                    std::cout << "[EVENT] Saving post-event frames..." << std::endl;
                    for (size_t i = 0; i < postEventFrames.size(); ++i) {
                        std::string filename = "event_frame_post_" + std::to_string(i) + ".jpg";
                        cv::imwrite(filename, postEventFrames[i]);
                    }

                    eventTriggered = false;
                    postEventFrames.clear();
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(inFlightMutex);
        inFlightRequests.erase(request);
        inFlightCondition.notify_all();
    }

    if (shuttingDown)
        return;

    request->reuse(libcamera::Request::ReuseBuffers);

    {
        std::lock_guard<std::mutex> lock(inFlightMutex);
        inFlightRequests.insert(request);
        camera->queueRequest(request);
    }
}

cv::Mat CameraManager::convertFrame(libcamera::FrameBuffer *buffer, const libcamera::StreamConfiguration &config) {
    const libcamera::FrameMetadata &metadata = buffer->metadata();
    if (metadata.status != libcamera::FrameMetadata::FrameSuccess)
        return {};

    const auto &planes = buffer->planes();
    if (planes.empty())
        return {};

    void *memory = mmap(nullptr, planes[0].length, PROT_READ, MAP_SHARED, buffer->planes()[0].fd.get(), 0);
    if (memory == MAP_FAILED) 
    {
        std::cerr << "Failed to mmap frame buffer." << std::endl;
        return {};
    }

    int width = config.size.width;
    int height = config.size.height;

    cv::Mat image(height, width, CV_8UC3, memory);
    cv::Mat copy;
    image.copyTo(copy);
    munmap(memory, planes[0].length);

    return copy;
}

cv::Mat CameraManager::getLatestFrame() 
{
    return frameBuffer.latest();
}

std::vector<cv::Mat> CameraManager::getFrameBuffer() 
{
    return frameBuffer.snapshot();
}

bool CameraManager::detectEvent(const cv::Mat& frame) {
    // Placeholder: return true if motion detected
    // For now, return false to keep system quiet
    return false;
}
