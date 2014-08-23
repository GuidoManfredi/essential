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
    FilesManager();
    // Load the path to all .png and .txt for an object
    Object loadObject(std::string folder_path);

  private:
    float readAngle(std::string pose_path);
    int isPoseFile (std::string filename, std::string &base_name);
    Pipeline2D pipe2d_;

};
