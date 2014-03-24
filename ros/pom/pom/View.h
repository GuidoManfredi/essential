#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "commons.h"

class View {
  public:
    View ();
    cv::Mat transform ();
    void write (cv::FileStorage& fs) const;
    void read (const cv::FileNode& node);

    cv::Mat view_;
    std::vector<cv::KeyPoint> keypoints_;
    std::vector<cv::Point3f> points3d_;
    cv::Mat descriptors_;
    cv::Mat R_, t_;
};

static void write(cv::FileStorage& fs, const std::string&, const View& x){
  x.write(fs);
}
static void read(const cv::FileNode& node, View& x, const View& default_value = View()){
  if(node.empty())
    x = default_value;
  else
    x.read(node);
}
