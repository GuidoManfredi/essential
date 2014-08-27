#pragma once

#include <fstream>
#include <iostream>
#include <numeric>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "Containers.h"
#include "Pipeline2D.h"

class FilesManager {
  public:
    FilesManager(cv::Mat K);
    // Load the path to all .png and .txt for an object
    Object loadObject(std::string folder_path);

  private:
    cv::Mat K_;

    float readAngle(std::string pose_path);
    int isPoseFile (std::string filename, std::string &base_name);

    void key2desc (std::vector<std::vector<keypointslist > > key, cv::Mat &desc);
    void key2kpts (std::vector<std::vector<keypointslist > > key, std::vector<cv::KeyPoint> &kpts);
    std::vector<cv::Point3f> depth2points (cv::Mat depth);

    Pipeline2D pipe2d_;

};
