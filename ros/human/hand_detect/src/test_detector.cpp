#include "Detector.h"

using namespace std;
using namespace cv;

void draw (Mat img, vector<Rect> hands);

const static Scalar colors[] =  { CV_RGB(0,0,255),
								    CV_RGB(0,128,255),
								    CV_RGB(0,255,255),
								    CV_RGB(0,255,0),
								    CV_RGB(255,128,0),
								    CV_RGB(255,255,0),
								    CV_RGB(255,0,0),
								    CV_RGB(255,0,255)} ;

// ./bin/test_detector /home/gmanfred/devel/ros/Vision_pipeline_new/icaro/hand_detect/cascades/palm.xml
int main (int argc, char** argv) {
	Detector det (argv[1]);
	
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return 1;

    Mat edges;
    namedWindow("hand",1);
    for(;;) {
        Mat frame;
        cap >> frame; // get a new frame from camera
        vector<Rect> hands;
        hands = det.detect (frame);
        draw (frame, hands);
        imshow("hand", frame);
        if(waitKey(30) >= 0) break;
    }
    
    return 0;
}

void draw (Mat img, vector<Rect> hands) {
	Scalar color;
	for (size_t i=0; i<hands.size(); ++i) {
    Scalar color = colors[i%8];
    rectangle (img, hands[i], color, 3, 8, 0 );
  }
}
