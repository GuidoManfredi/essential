#ifndef DEF_CAROD_TRAINER
#define DEF_CAROD_TRAINER

#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Matcher.h"
#include "Object.h"
//#include "file_tools.cpp"

#define NUM_TRAIN_IMG 4

namespace bf = boost::filesystem;

// Object corners in current image
static std::vector<cv::Point2f> corners;
static void mouse_cb( int event, int x, int y, int flags, void* param )
{
  // If button released
  if (event == CV_EVENT_LBUTTONUP)
  {
    if (corners.size () >= 4)
      corners.clear ();

    corners.push_back ( cv::Point2f(x, y));
    std::cout << "Corner " << corners.size () << " saved." << std::endl;
  }
}

class Trainer
{
 public :
  /**
  * Train a directory containing one sub-directory per object.
  **/
  void train (std::string dirname);
  /**
  * Read a file containing the width, height and depth dimensions
  * of the object.
  **/
  bool read_info (std::string dirname);
  /**
  * Load the four training images, warp them so the camera is centered
  * in each face and save back the warped images.
  **/
  bool load_images (std::string dirname);
  /**
  * Detect features on each image and save corresponding pairs of
  * descriptors/3D points.
  **/
  void train_object ();
  /**
  * Warp an image so the camera is a at the center of the object of
  * interesset, delimited by the corners and with dimensions width and
  * height.
  **/
  void rect_img (cv::Mat in, cv::Mat &out,
                 std::vector<cv::Point2f> corners, float width, float height);

  /**
  * Save last object in pcd format
  **/
  void save_objects (std::string name);
  Object get_object_by_name (std::string name);

  Trainer ();
  ~Trainer ();
 private:
  Matcher M;
  std::vector<Object> _objects;
  // input
  std::string _name;
  float _w, _h, _d;
  std::vector<cv::Mat> _img;

	// output
  cv::Mat	_descriptors;
  std::vector<cv::Point3d> _p3d;
};

// TODO : marque trained objects as trained so they are not processed again.

#endif // DEF_CAROD_TRAINER
