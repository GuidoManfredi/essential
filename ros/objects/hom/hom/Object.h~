#pragma once

#include <opencv2/core/core.hpp>

class Object {
  public:
    Object();
    void addFace(cv::Mat image, float scale);
    
    int load(std::string path);
    int save(std::string path);
    
    
  private:
    Face faces_;
    cv::Mat Tfo; // transform face->origin
};
