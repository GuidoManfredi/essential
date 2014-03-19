#include "Viewer.h"

using namespace cv;

int Viewer::loop ()
{
  if (!_viewer->wasStopped ())
  {

    _viewer->spinOnce (100);
    //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
    return 1;
  }

  return 0;
}

void Viewer::add_transfomation (Matx34d P)
{
  _viewer->removeCoordinateSystem ();
  //Eigen::Vector3f t (P(0,3), P(1,3), P(2,3));
  Eigen::Vector3f t (1, 1, 1);
  Eigen::Matrix3f R;
  R << P(0,0), P(0,1), P(0,2), P(1,0), P(1,1), P(1,2), P(2,0), P(2,1), P(2,2); // this is the good, god knows why
  //R << P(0,0), P(1,0), P(2,0), P(0,1), P(1,1), P(2,1), P(0,2), P(1,2), P(2,2);

  Eigen::Affine3f T = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

  _viewer->addCoordinateSystem (1.0, T);
}

void Viewer::add_cloud (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  _viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "object");
  _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object");
}

Viewer::Viewer (): _viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"))
{
  _viewer->setBackgroundColor (0, 0, 0);
  _viewer->addCoordinateSystem (1.0);
  _viewer->initCameraParameters ();
}

Viewer::~Viewer ()
{}
