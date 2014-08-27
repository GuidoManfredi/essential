#include <iostream>

#include "HandSeg.h"

using namespace std;
using namespace cv;

void testGesture () {
    HandSeg seg;
    Mat img = imread("/home/gmanfred/devel/datasets/hands/hand_binary.png", CV_LOAD_IMAGE_GRAYSCALE);
    seg.gesture (img);
}

int main() {

    testGesture();

    return 0;
}
