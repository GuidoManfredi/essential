#pragma once

#include <opencv2/core/core.hpp>

class View {
  public:
    View ();
    void write (cv::FileStorage& fs) const;
    void read (const cv::FileNode& node);

    int number_points_;
    std::vector<cv::Point3f> points3d_;
    cv::Mat descriptors_;
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
