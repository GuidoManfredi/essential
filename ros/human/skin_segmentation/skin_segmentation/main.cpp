#include "SkinSeg.h"

using namespace std;
using namespace cv;

int main()
{
    SkinSeg sg(3);
    sg.train("/home/gmanfred/devel/datasets/hands/EM/sample.jpg");
    Mat test = imread("/home/gmanfred/devel/datasets/hands/EM/test.jpg", CV_LOAD_IMAGE_COLOR);
    Mat mask;
    sg.segment(test, mask);

    imshow("Debug", mask); waitKey(0);
    return 0;
}
