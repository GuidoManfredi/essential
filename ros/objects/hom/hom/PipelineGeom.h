#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Face.h"
//#include "Pipeline2D.h"
#include "PipelineGpu2D.h"

class PipelineGeom {
  public:
    PipelineGeom();
    bool computeHomography (std::vector<cv::KeyPoint> kpts1, std::vector<cv::KeyPoint> kpts2,
                                std::vector<cv::DMatch>& matches,
                                cv::Mat &homography);
    bool computePose(Face frame, Face face,
                      cv::Mat &H, std::vector<cv::DMatch> &inliers);
  private:
    void keypoints2points(std::vector<cv::KeyPoint> kpts1, std::vector<cv::KeyPoint> kpts2, std::vector<cv::DMatch> matches,
                            std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2);
    void keypoints2points(std::vector<cv::KeyPoint> kpts, std::vector<cv::Point2f> &points);
    void points2keypoints(std::vector<cv::Point2f> points, std::vector<cv::KeyPoint> &kpts);
    void homography2transform(cv::Mat H, cv::Mat &P);

    //Pipeline2D pip_;
    PipelineGpu2D pip_;
};
