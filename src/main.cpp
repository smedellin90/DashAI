// main.cpp
#include "CameraManager.hpp"
// #include "EventDetector.hpp"
// #include "MockIMU.hpp"
#include <thread>
#include <chrono>

int main() 
{
    CameraManager camera;
    // MockIMU imu;
    // EventDetector detector(&imu);

    camera.startCapture();
    // detector.startMonitoring(camera);

    // Main thread sleeps while others work
    while (true) 
    {
        cv::Mat frame = camera.getLatestFrame();
        if (!frame.empty())
            std::cout << "Captured Frame size: " << frame.cols << "x" << frame.rows << std::endl;
        else
            std::cout << "No frame captured yet" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
