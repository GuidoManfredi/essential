#include "SkinSeg.h"

using namespace std;
using namespace cv;

void testStatic() {
    SkinSeg sg(3);
    sg.train("/home/gmanfred/devel/datasets/hands/EM/sample.jpg");

    //Mat test = imread("/home/gmanfred/devel/datasets/hands/EM/test1.jpg", CV_LOAD_IMAGE_COLOR);
    Mat test = imread("/home/gmanfred/devel/datasets/hands/EM/test2.jpg", CV_LOAD_IMAGE_COLOR);
    Mat mask;
    sg.segment(test, mask);
    imshow("Debug", mask); waitKey(0);
}

void testVideo() {
    SkinSeg sg(3);
    sg.train("/home/gmanfred/devel/datasets/hands/EM/sample2.jpg");

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return;

    for(;;) {
        Mat frame;
        cap >> frame;

        Mat mask;
        sg.segment(frame, mask);
        imshow("Mask", mask);
        imshow("Frame", frame); waitKey(1);
    }
}

int main()
{
    //testStatic();
    testVideo();

    return 0;
}
