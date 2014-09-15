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
    FilesManager(cv::Mat K, Pipeline2D* pipe);
    void setFeatures(Feature ft);
    int getFeatures();
    // Load the path to all .png and .txt for an object
    Object loadObject(std::string folder_path);
    int getNumFolders(std::string path);

  private:
    cv::Mat K_;

    float readAngle(std::string pose_path);
    cv::Point2f readOrigin (std::string path);
    int isPoseFile (std::string filename, std::string &base_name);

    std::vector<cv::Point3f> depth2points (cv::Mat depth, cv::Point2f origin);

    Pipeline2D* pipe2d_;
    Feature features_;
};
