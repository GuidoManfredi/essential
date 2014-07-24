#pragma once

#include <opencv2/core/core.hpp>
#include "Face.h"

class HOM {
  public:
    //int createObjectFromList(std::string list_path);
    //int createObject(std::vector<cv::Mat> images);
    //std::vector<Face> createFacesFromList(std::string list_path);
    std::vector<Face> createFacesFromList(std::string list_path);
    Face createFace(std::string image_path);
    Face createFace(cv::Mat image);
    
    int save(std::vector<Face> faces);
    int save(Face face);

  private:
    int ReadList(std::string list_path,
                 std::vector<std::string> paths);
};
