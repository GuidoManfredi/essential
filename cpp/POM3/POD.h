#pragma once

#include <opencv2/highgui/highgui.hpp>

#include "Pipeline2D.h"
#include "PipelineGeom.h"

#include "Object.h"
#include "Face.h"
// POD : Planar Object Detector
class POD
{
  public:
    POD ();
    bool process (cv::Mat image);
    bool findFace (Face face);
    bool findView (std::vector<cv::KeyPoint> train_keypoints, cv::Mat train_descriptors);

    void addFace (Face face);

  private:
    void showMatches (cv::Mat image1, std::vector<cv::KeyPoint> keypoints1,
                      cv::Mat image2, std::vector<cv::KeyPoint> keypoints2,
                      std::vector<cv::DMatch> matches);

    Pipeline2D pipeline2d_;
    PipelineGeom pipelineGeom_;

    std::vector<Face> faces_;

    cv::Mat current_image_;
    std::vector<cv::KeyPoint> current_keypoints_;
    cv::Mat current_descriptors_;
    cv::Mat current_bow_;
    std::vector<cv::DMatch> current_matches_;
    int closest_view_;
};
