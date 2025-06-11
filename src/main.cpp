#include "CameraManager.hpp"
#include <iostream>
#include <chrono>
#include <thread>
// #include <opencv2/opencv.hpp>

int main() {
    CameraManager camera;
    camera.startCapture();

    std::cout << "[INFO] Camera capture started. Checking for frames..." << std::endl;

    while (true) {
        cv::Mat frame = camera.getLatestFrame();

        if (!frame.empty()) {
            std::cout << "[FRAME] Captured: " << frame.cols << "x" << frame.rows << std::endl;

            // Optional: show frame (comment out if headless)
            /*
            cv::imshow("Live Feed", frame);
            if (cv::waitKey(1) == 27) break; // ESC to exit
            */
        } else {
            std::cout << "[WAIT] No frame yet..." << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
