#include "Visualizer.h"

using namespace cv;
using namespace std;

int main (int argc, char** argv) {

	Mat K(3,3,CV_64F);
    Mat d(5,1,CV_64F);
    FileStorage r_fs;
    r_fs.open ("/home/gmanfred/.ros/camera_info/webcam_gilgamesh_opencv.yml", cv::FileStorage::READ);
    r_fs["camera_matrix"]>>K;
    r_fs["distortion_coefficients"]>>d;
    r_fs.release ();

	Mat img = imread ("/home/gmanfred/Desktop/test3.jpg");
	cv::Size size(img.cols, img.rows);
	Visualizer display("test", img.size(), K);
	/*
	Mat pose = (Mat_<double>(4,4) << 0.904649, 0.378819, 0.195208, -0.243246,
																	 -0.395126, 0.574008, 0.71721, 0.0737445,
																	 0.159642, -0.725955, 0.668957, -4.09743,
																	 0.0, 0.0, 0.0, 1.0);
	*/
	Mat pose (4, 4, CV_64F);
	pose.at<double>(0,0) = 0.904649;	pose.at<double>(0,1) = 0.378819;	pose.at<double>(0,2) = 0.195208;	pose.at<double>(0,3) = -0.243246;
	pose.at<double>(1,0) = -0.395126;	pose.at<double>(1,1) = 0.574008;	pose.at<double>(1,2) = 0.71721;	pose.at<double>(1,3) = 0.0737445;
	pose.at<double>(2,0) = 0.159642;	pose.at<double>(2,1) = -0.725955;	pose.at<double>(2,2) = 0.668957;	pose.at<double>(2,3) = -4.09743;
	pose.at<double>(3,0) = 0.0;	pose.at<double>(3,1) = 0.0;	pose.at<double>(3,2) = 0.0;	pose.at<double>(3,3) = 1.0;
	display.updateBackground(img);
	display.updatePose(pose);
	display.updateWindow();
	
    cv::waitKey(0);
    
    return 0;
}

