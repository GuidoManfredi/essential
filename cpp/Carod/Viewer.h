#ifndef DEF_CAROD_VIEWER
#define DEF_CAROD_VIEWER

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "opencv2/core/core.hpp"

class Viewer
{
public:
  int loop ();
  // PCL : X = red, Y = green, Z = blue
  void add_transfomation (cv::Matx34d P);
  void add_cloud (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

  Viewer ();
  ~Viewer ();

  boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
};

#endif
