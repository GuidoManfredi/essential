#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

class SkinSeg {
  public:
    SkinSeg(int num_gaussians);
    void train(std::string model_path);
    void segment(cv::Mat frame, cv::Mat &mask);
  private:
    void train(cv::Mat model);
    // Transform image to a float column with each pixel in a row.
    void image2column(cv::Mat image, cv::Mat &column);

    cv::EM model_;
};
