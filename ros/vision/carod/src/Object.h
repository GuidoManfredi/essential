#ifndef DEF_CAROD_OBJECT
#define DEF_CAROD_OBJECT

#include <iostream>
#include <fstream>
#include <string>

/*
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Object
{
 public :
  Object ();
  Object (std::string name, float w, float h, float d);
  ~Object ();
  // int n : numero of image. 0 is the front, 1 is right side,
  //                          2 is the back, 3 is left side.
  // cv::Keypoints kpts contains a point position
  void add_p3d_from_kpts (int n, std::vector<cv::KeyPoint> kpts,
                          float img_w, float img_h);
  void add_img (cv::Mat img);
  void add_descs (cv::Mat descs);

  float get_r_from_p3d (int p3d_idx);
  float get_g_from_p3d (int p3d_idx);
  float get_b_from_p3d (int p3d_idx);

  std::vector<cv::Point3d> get_p3d ();
  std::string get_name ();

  //void save_to_pcd (std::string path);
  void save_descriptors (std::string path);
  void save_descriptors_with_key_points (std::string path);

  // Each row of this matrix is a descriptor. There are as many rows as 3D
  // points.
  std::vector<cv::Mat> _img;
  std::vector<cv::KeyPoint> _kpts;
  std::vector<cv::Mat>	_descriptors;
  std::vector<cv::Point3d> _p3d;

  std::string _name;
  float _h, _w, _d;
};

#endif // DEF_CAROD_OBJECT
