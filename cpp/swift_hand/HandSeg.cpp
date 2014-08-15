#include "HandSeg.h"

using namespace std;
using namespace cv;

HandSeg::HandSeg() {

}

void HandSeg::getHands(vector<Mat> images,
               vector<Mat> &hands) {
    hands.clear();
    for (size_t i = 0; i < images.size(); ++i) {
        Mat tmp_hand = getHand(images[i]);
        hands.push_back(tmp_hand);
    }
}

Mat HandSeg::getHand(Mat image) {
    Mat min_hsv = (Mat_<int>(1,3) << 0, 10, 0);
    Mat max_hsv = (Mat_<int>(1,3) << 20, 150, 255);
    // Get skin mask
    Mat mask;
    segmentSkinHSV(image, min_hsv, max_hsv, mask);
    // Get skin image from mask
    Mat skin;
    image.copyTo(skin, mask);
    // Get boundign box around hand
    Rect bb = getHandBox(mask);
    // Get image in bounding box
    Mat roi = skin(bb);
    Mat output;
    roi.copyTo(output);
    //resize(output, output, Size(100,100));
    return output;
}

cv::Rect HandSeg::getHandBox (cv::Mat mask) {
    vector<vector<Point> > contours;
    findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    float max_area = 0;
    int max_i = 0;
    for(size_t i = 0; i < contours.size(); ++i) {
        float tmp_area = contourArea(contours[i]);
        if (tmp_area > max_area) {
            max_area = tmp_area;
            max_i = i;
        }
    }

    return boundingRect(contours[max_i]);
}

////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void HandSeg::segmentSkinHSV(Mat image, Mat min_hsv, Mat max_hsv,
                           Mat &mask) {
    Mat blurred;
    GaussianBlur(image, blurred, Size(5,5), 0);
    Mat hsv;
    cvtColor(blurred, hsv, CV_BGR2HSV);
    inRange(hsv, min_hsv, max_hsv, mask);
}

