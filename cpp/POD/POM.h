#ifndef POD_POM_H_
#define POD_POM_H_

#include "Pipeline.h"
#include "PlanarObject.h"

// Planar Object Modeling
class POM
{
 public:
  POM ();
  void img2object (cv::Mat image,
                    PlanarObject &object);
  void img2object (cv::Mat image, std::vector<cv::Point2f> corners2d,
                    PlanarObject &object);
 private:
  Pipeline pipeline;
};

#endif // POD_POM_H_
