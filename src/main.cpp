#include "CameraManager.hpp"
#include "CameraConfig.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/imgcodecs.hpp>  // for imwrite
#include <string.h>

std::string getTimestampedFilename(const std::string &dir = "frames/", const std::string &prefix = "test_frame", const std::string &ext = ".jpg") {
    auto now = std::time(nullptr);
    std::tm *tm = std::localtime(&now);

    std::ostringstream oss;
    oss << dir << prefix << "_"
        << std::put_time(tm, "%Y-%m-%d_%H-%M-%S")
        << ext;
    return oss.str();
}

int main() {
    CameraConfig config {
        .preEventDuration = std::chrono::milliseconds(4000),
        .postEventDuration = std::chrono::milliseconds(5000),
        .bufferCapacity = 500,
    };

    CameraManager cam = CameraManager(config);
    cam.startCapture();

    std::cout << "[INFO] Camera capture started. Checking for frames..." << std::endl;

    int framesCaptured = 0;

    while (framesCaptured < 100) {
        if (cv::Mat frame = cam.getLatestFrame(); !frame.empty()) {
            std::cout << "[FRAME " << framesCaptured << "] "
                      << "Resolution: " << frame.cols << "x" << frame.rows
                      << ", Mean Pixel: " << cv::mean(frame) << std::endl;

            if (framesCaptured == 0) {
                // Save first frame to disk
                const std::string filename = getTimestampedFilename();
                cv::imwrite(filename, frame);
                std::cout << "[INFO] Saved " << filename << std::endl;
            }

            framesCaptured++;
        } else {
            std::cout << "[WAIT] No frame yet..." << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[INFO] Done capturing. Exiting." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    return 0;
}
