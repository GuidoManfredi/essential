#include "Object.h"

using namespace std;
using namespace cv;

double rand(double min, double max)
{
  return min + (max - min) * double(rand()) / RAND_MAX;
}

void Object::add_p3d_from_kpts (int n, std::vector<cv::KeyPoint> kpts,
                                float img_w, float img_h)
{
  double x, y, z, fixed = 0;
  _kpts.push_back (kpts);
//  srand(time(0));
//  double noise = 10;
  //Point3f origine(_w/2, _h/2, _d/2);
  for (size_t i=0; i<kpts.size (); ++i)
  {
    // i is in [0,3] depending on object face
    switch (n) {
      // front image
      case 0:{
        x = _d/2.0;
        y = (_w/img_w)*kpts[i].pt.x - _w/2.0;
        z = _h/2.0 - (_h/img_h)*kpts[i].pt.y;
        break;
      }
      // left side
      case 1:{
        x = _d/2.0 - (_d/img_w)*kpts[i].pt.x;
        y = _w/2.0;
        z = _h/2.0 - (_h/img_h)*kpts[i].pt.y;
        break;
      }
      // back image
      case 2:{
        x = - _d/2.0;
        y = _w/2.0 - (_w/img_w)*kpts[i].pt.x;
        z = _h / 2.0 - (_h/img_h)*kpts[i].pt.y;
        break;
      }
      // right side
      case 3:{
        x = (_d/img_w)*kpts[i].pt.x - _d/2.0;
        y = - _w/2.0;
        z = _h/2.0 - (_h/img_h)*kpts[i].pt.y;
        break;
      }
      default:
        cout << "Error: add_p3d_from_kpts: " << n << "is not a possible case." << endl;
    }

//    double nx = rand(-noise, +noise);
//    double ny = rand(-noise, +noise);
//    double nz = rand(-noise, +noise);
    //cout << "Adding new point "<< n << " " << x << " " << y << " " << z << endl;
    //_p3d.push_back (Point3f (x+nx, y+ny, z+nz));
    _p3d.push_back (Point3d (x, y, z));
  }
}

void Object::add_img (Mat img)
{
  _img.push_back (img);
}

void Object::add_descs (cv::Mat descs)
{
  cout << descs.size () << endl;
  _descriptors.push_back (descs);
}

void Object::save_to_pcd (string path)
{
  // create cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  cloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);

  //cout << _kpts.size () << endl;
  for (size_t i=0, n=0; i<_kpts.size (); ++i)
  {
    //cout << _kpts[i].size () << endl;
    for (size_t j=0; j<_kpts[i].size (); ++j, ++n)
    {
      pcl::PointXYZRGB pt;
      pt.x = _p3d[n].x;
      pt.y = _p3d[n].y;
      pt.z = _p3d[n].z;
      Vec3b colour = _img[i].at<Vec3b>(Point(_kpts[i][j].pt.x, _kpts[i][j].pt.y));

      uint8_t r(colour[2]), g(colour[1]), b(colour[0]);
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      pt.rgb = *reinterpret_cast<float*>(&rgb);

      cloud->push_back(pt);
    }
  }

  cloud->width = _p3d.size ();
  cloud->height = 1;

  std::ostringstream pcd_path;
  pcd_path << path << "/" << _name << ".pcd";
  pcl::io::savePCDFileASCII(pcd_path.str(), *cloud);
}

void Object::save_descriptors (string path)
{
  cv::FileStorage fs;
  std::ostringstream descs_path;
  descs_path << path << "/" << _name << ".yml";
  fs.open (descs_path.str(), cv::FileStorage::WRITE);
  fs << "descriptors" << _descriptors;
  fs.release ();
}

void Object::save_descriptors_with_key_points (string path)
{
  ofstream file;
  path = path + "/" + _name + "_debug.txt";
  file.open (path.c_str());
  int offset = 0;
  for(int i=0; i<_kpts.size(); ++i)
  {
    for(int j=0; j<_kpts[i].size(); ++j)
      file << _p3d[offset+j].x << " " << _p3d[offset+j].y << " " << _p3d[offset+j].z << " "
          << _kpts[i][j].pt.x << " " << _kpts[i][j].pt.y << endl;
    offset += _kpts[i].size();
  }


  file.close();
}

Object::Object (string name, float w, float h, float d):
_w(w), _h(h), _d(d), _name(name)
{
}

Object::~Object ()
{}
