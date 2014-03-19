#include <iostream>
#include "Trainer.h"
#include "Detector.h"
#include "Viewer.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  Mat K(3,3,CV_32FC1);
  Mat d(5,1,CV_32FC1);
  FileStorage r_fs;
  r_fs.open ("/home/gmanfred/.ros/camera_info/webcam_gilgamesh_opencv.yml", cv::FileStorage::READ);
  r_fs["camera_matrix"]>>K;
  r_fs["distortion_coefficients"]>>d;
  r_fs.release ();
  Trainer t;
  t.train ("/home/gmanfred/devel/these/projects/Carod/training_set");

  string object_name1 = "cereales_nesquik";
  string object_name2 = "couscous";
//  stirng object_name = "jus_orange";

  Detector D (K, d);
  Object nesquik = t.get_object_by_name(object_name1);
  Object couscous = t.get_object_by_name(object_name2);
  //D.train_kpts_descs(obj._kpts, obj._descriptors);
  D.train_descs(nesquik._descriptors);
  D.train_descs(couscous._descriptors);

  //Mat img = imread ("/home/gmanfred/devel/these/projects/Carod/training_set/" + object_name + "/warp_2.jpg");
  //Mat img = imread ("/home/gmanfred/devel/these/projects/Carod/training_set/" + object_name + "/view_0.jpg");
  Mat img = imread ("/home/gmanfred/Pictures/nesquik_facile.jpg");

  Matx34d P;
  P = D.compute_pose(img, nesquik._p3d);

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/gmanfred/devel/these/projects/Carod/training_set/" + object_name + "/" + object_name + ".pcd", *cloud);

  /*
  double elapTime;
  clock_t beginT, endT;

  VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if succeeded
      return -1;

  char c;
  Matx34d P;
  Viewer v;
  //namedWindow("Object",1);
  for(;;)
  {
    v.loop();
    //c = waitKey(0);
    waitKey(1);
    //if (c == ' '){
    Mat img;
    cap >> img; // get a new frame from camera
    beginT = clock();
    P = D.compute_pose(img, obj._p3d);
    v.add_transfomation(P);
    endT = clock();
    cout << ((double)(endT - beginT)*1000)/CLOCKS_PER_SEC << endl;
    imshow("Object", img);
    //}
    if (c == 'q')
      break;
  }
  */
  return 0;
}

// True poses :
/* W0 = [0 1 0
         0 0 -1
         -1 0 0]
   W1 = [-1 0 0
          0 0 -1
          0 -1 0]
   W2 = [0 -1 0
         0 0 -1
         1 0 0]
   W3 =[1 0 0
        0 0 -1
        0 1 0]
*/
// TODO : multiple identical points appear in the trained 3D points with SIFT.
//        Fix that ! Bitch ! => Known problem in opencv.
// TODO : fix broken save_to_pointcloud.
// TODO : visualize matches between current image and 3d pts (or highlights
//        selected 3D points on cloud viewer).
