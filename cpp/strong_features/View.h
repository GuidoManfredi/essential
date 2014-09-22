#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ASIFT/demo_lib_sift.h"

class View
{
  public:
    View ();
    View (const View &view);

    void write (cv::FileStorage& fs) const;
    void read (const cv::FileNode& node);

    cv::Mat image_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

    std::vector<std::vector<keypointslist> > keys_;
    std::vector<cv::Point3f> points_;
    int tilt_;
    float angle_;
};

//These write and read functions must exist as per the inline functions in operations.hpp
static void write(cv::FileStorage& fs, const std::string&, const View& x){
  x.write(fs);
}
static void read(const cv::FileNode& node, View& x, const View& default_value = View()){
  if(node.empty())
    x = default_value;
  else
    x.read(node);
}
