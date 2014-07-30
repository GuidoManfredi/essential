#pragma once

#include <opencv2/core/core.hpp>
//#include "Pipeline2D.h"
#include "PipelineGpu2D.h"
#include "Face.h"
#include "FilesManager.h"

class HOM {
  public:
    Face createFace(std::string image_path, int id);
    Face createFace(cv::Mat image, int id);
    void saveFace (Face face, std::string path);

  private:
    //Pipeline2D pip_;
    PipelineGpu2D pip_;
};
