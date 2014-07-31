#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Object.h"
#include "Pipeline2D.h"
#include "PipelineGeom.h"

class POD
{
  public:
    POD ();
    void setObject (Object object);
    void loadObject (std::string path);
    //void loadObjectFromList (std::string list_path);
    void setIntrinsic (cv::Mat K);
    void loadIntrinsic (std::string path);
    std::string objectName (int object_number);
    void process (const cv::Mat image, std::vector<cv::Mat> &poses);
    void process (View current, std::vector<cv::Mat> &poses);

  private:
    View createView (cv::Mat image);
    void match (View current, Object object, std::vector<cv::DMatch> &matches);
    cv::Mat computePose (View current, Object object, std::vector<cv::DMatch> matches);
    void match2points (std::vector<cv::KeyPoint> keypoints, std::vector<cv::Point3f> points, std::vector<cv::DMatch> matches,
                        std::vector<cv::Point2f> &points2d, std::vector<cv::Point3f> &points3d);

    cv::Mat K_;
    std::vector<Object> objects_;
    Pipeline2D pipeline2d_;
    PipelineGeom pipelineGeom_;
};
