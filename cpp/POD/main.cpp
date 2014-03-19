#include <iostream>
#include <assert.h>

#include "POD.h"
#include "POM.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  //assert (argc == 2 && "Usage : POD image_path");

  // Creating model
  char * image_path = "/home/gmanfred/Pictures/spidey.jpg";
  PlanarObject object;
  Mat img = imread (image_path, CV_LOAD_IMAGE_GRAYSCALE);
  assert (img.data && "Could not read image");

  POM modeler;
  modeler.img2object (img, object);

  // Load intrinsic parameters
  Mat K(3,3,CV_32FC1);
  Mat d(5,1,CV_32FC1);
  FileStorage r_fs;
  r_fs.open ("/home/gmanfred/.ros/camera_info/webcam_gilgamesh_opencv.yml", cv::FileStorage::READ);
  r_fs["camera_matrix"]>>K;
  r_fs["distortion_coefficients"]>>d;
  r_fs.release ();

  // Initialize detector
  POD detector (K);
  detector.train (object);

  // Detect on video stream
  Mat frame;
  VideoCapture capture(0); // open the default camera
  assert(capture.isOpened() && "Couldn't open default camera");  // check if succeeded
  for( ; ; ) {
    capture >> frame;
    assert(frame.data && "Empty frame captured from video");
    imshow("w", frame);
    waitKey(20); // waits to display frame
  }
  waitKey(0);

  return 0;
}
