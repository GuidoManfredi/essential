#pragma once
#include <fstream>
#include <opencv2/ml/ml.hpp>

class SwiftHand {
  public:
    SwiftHand();
    void tuneParameters (cv::Mat train_mat, cv::Mat train_labels,
                         cv::Mat test_mat, cv::Mat test_labels,
                         float min_c, float step_c, float max_c,
                         float min_gamma, float step_gamma, float max_gamma);
    void train (cv::Mat train_mat, cv::Mat train_labels, float C, float gamma);
    cv::Mat confusionMatrix (cv::Mat test_mat, cv::Mat labels, int max_label);

  private:
    CvSVM mySVM_;
};
