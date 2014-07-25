#pragma once

#include <opencv2/core/core.hpp>
#include "Face.h"
#include "FilesManager.h"

class HOM {
  public:
    Face createFace(std::string image_path, int id);
    Face createFace(cv::Mat image, int id);
    
    int save(std::vector<Face> faces);
    int save(Face face);

  private:
    int ReadList(std::string list_path,
                 std::vector<std::string> paths);
};
