#include "HandSeg.h"

using namespace std;
using namespace cv;

HandSeg::HandSeg () {

}

int HandSeg::getGesture (Mat depth, Mat mask) {
    Mat hist = getZhistogram (depth, mask);
    Mat crop = cropHand(hist);
    Mat display;
    return gesture(crop, display);
}

Mat HandSeg::getZhistogram(Mat depth, Mat mask) {
    int histSize = 256;
    float range[] = { 0, 256 };
    const float* histRange = { range };
    bool uniform = true; bool accumulate = false;
    Mat hist;
    calcHist(&depth, 1, 0, mask, hist, 1, &histSize, &histRange, uniform, accumulate );
    return hist;
}

cv::Mat HandSeg::cropHand (cv::Mat z_histogram) {

}

// Take mask as entry, return the corresponding gesture
int HandSeg::gesture (Mat mask, Mat &display) {
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours (mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    drawContours (display, contours, -1, Scalar(0, 255, 0));

    vector<vector<int> > hull (contours.size());
    vector<vector<Vec4i> > defects (contours.size());
    for( int i = 0; i < contours.size(); i++ ) {
        convexHull (Mat(contours[i]), hull[i]);
        if (contours[i].size() > 3) {
            convexityDefects (contours[i], hull[i], defects[i]);
        }
    }

    for( int i = 0; i < contours.size(); i++ ) {
        cout << defects[i].size() << endl;
    }

    return 0;
}
