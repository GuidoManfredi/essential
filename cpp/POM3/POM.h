#pragma once

#include "Pipeline2D.h"
#include "SyntheticViewGenerator.h"

#include "Object.h"
#include "Face.h"

class POM
{
  public:
	POM();
	Object createObjectWithSynthetic (std::vector<cv::Mat> images);
    Face createFaceWithSynthetic (cv::Mat image);
    Face createFace (std::vector<cv::Mat> views);

    //bool saveFace (Face face);
    //Face loadFace (std::string filepath);

  private:
    SyntheticViewGenerator view_generator_;
    Pipeline2D features_pipeline_;
};

