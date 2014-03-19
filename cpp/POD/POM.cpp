#include "POM.h"

using namespace cv;

POM::POM () {

}

void POM::img2object (Mat image,
                        PlanarObject &object) {
  const float w = image.cols;
  const float h = image.rows;
  std::vector<cv::Point2f> corners2d(4);
  corners2d[0] = cv::Point2f(0,0);
  corners2d[1] = cv::Point2f(w,0);
  corners2d[2] = cv::Point2f(w,h);
  corners2d[3] = cv::Point2f(0,h);
  img2object (image, corners2d, object);
}

void POM::img2object (Mat image, std::vector<cv::Point2f> corners2d,
                        PlanarObject &object) {
  // Store original image in pattern structure
  object.size = cv::Size(image.cols, image.rows);
  object.frame = image.clone();
  pipeline.getGray(image, object.gray);

  // Image dimensions
  const float w = image.cols;
  const float h = image.rows;

  // Normalized dimensions:
  const float maxSize = std::max(w,h);
  const float unitW = w / maxSize;
  const float unitH = h / maxSize;

  // Build 2d and 3d contours (3d contour lie in XY plane since it's planar)
  assert (corners2d.size() == 4 && "corners size != 4");
  object.corners2d.resize(4);
  object.corners2d[0] = corners2d[0];
  object.corners2d[1] = corners2d[1];
  object.corners2d[2] = corners2d[2];
  object.corners2d[3] = corners2d[3];

  object.corners3d.resize(4);
  object.corners3d[0] = cv::Point3f(-unitW, -unitH, 0);
  object.corners3d[1] = cv::Point3f( unitW, -unitH, 0);
  object.corners3d[2] = cv::Point3f( unitW,  unitH, 0);
  object.corners3d[3] = cv::Point3f(-unitW,  unitH, 0);

  pipeline.extractFeatures(object.gray, object.keypoints, object.descriptors);
}
