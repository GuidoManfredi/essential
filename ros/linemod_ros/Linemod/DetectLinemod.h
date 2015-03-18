#pragma once

#include <pcl/io/pcd_io.h>
//#include <pcl/recognition/linemod/line_rgbd.h>

#include "common.h"

class DetectLinemod {
  public:
    DetectLinemod(float grad_mag_thresh, float detect_thresh);

    bool loadCloud (const std::string & filename, PointCloudXYZRGBA & cloud);
    bool loadModel (const std::vector<std::string> & lmt_filenames);
    int detect (PointCloudXYZRGBA::Ptr cloud);

  private:
    //pcl::LineRGBD<PointType> line_rgbd_;
};
