#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/recognition/linemod.h>

#include "common.h"

class TrainLinemod {
  public:
    TrainLinemod();

    bool loadCloud (const std::string & filename, PointCloudXYZRGBA & cloud);
    void trainTemplate (const PointCloudXYZRGBA::ConstPtr & input, pcl::LINEMOD & linemod);
    void trainTemplate (const PointCloudXYZRGBA::ConstPtr & input, const std::vector<bool> &foreground_mask,
                        pcl::LINEMOD & linemod);

  private:
};


