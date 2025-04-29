#pragma once

#include <opencv2/opencv.hpp>

namespace wro {
class Camera {
   public:
    Camera();
    ~Camera();
    void run();

   private:
    cv::VideoCapture camera{};

    std::vector<DetectedBlock> detectedBlocks(const cv::Mat& frame);
    int calculateSteeringAngle(int frameWidth, int targetXHorizontal, int currentXHorizontal);
    void run();
};
}