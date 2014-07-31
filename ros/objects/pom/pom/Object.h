#pragma once

#include "View.h"

class Object {
  public:
    Object ();
    void addView (std::vector<cv::Point3f> points3d, cv::Mat descriptors);
    void getDescriptors (cv::Mat &descriptors);
    void getPoints (std::vector<cv::Point3f> &points3d);
    std::string name();

    void write (cv::FileStorage& fs) const;
    void read (const cv::FileNode& node);

    int number_views_;
    std::vector<View> views_;
    std::string name_;
};

static void write(cv::FileStorage& fs, const std::string&, const Object& x){
  x.write(fs);
}
static void read(const cv::FileNode& node, Object& x, const Object& default_value = Object()){
  if(node.empty())
    x = default_value;
  else
    x.read(node);
}
