#include "../gridloc/pipeline2D.h"

using namespace std;
using namespace cv;

Pipeline2D::Pipeline2D () {}

void Pipeline2D::color2gray (Mat image, Mat &gray) {
    assert(!image.empty() && "pipeline2d: color2gray: Image empty");
    if (image.channels()  == 3)
        cvtColor(image, gray, CV_BGR2GRAY);
    else if (image.channels() == 4)
        cvtColor(image, gray, CV_BGRA2GRAY);
    else if (image.channels() == 1)
        gray = image;
}

int Pipeline2D::detectCircles (Mat image, vector<KeyPoint> &circles) {
    vector<KeyPoint> all_circles;
    findCircles (image, circles);
    //removeNonRemarquableCircles (all_circles, circles);
}

void Pipeline2D::drawCircles (Mat &image, std::vector<KeyPoint> circles) {
    for( size_t i = 0; i < circles.size(); i++ ) {
        int radius = circles[i].size;
        // circle center
        circle(image, circles[i].pt, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle(image, circles[i].pt, radius, Scalar(0,0,255), 3, 8, 0 );
    }
}

////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
int Pipeline2D::findCircles (Mat image, std::vector<KeyPoint> &keypoints) {
    // Blurring not necessary, we are working in industrial space with few texture.
    //Mat blurred;
    //GaussianBlur(image, blurred, Size(9, 9), 2, 2);

    vector<Vec3f> circles;
    double min_dist = 1;
    //HoughCircles ( blurred, circles, CV_HOUGH_GRADIENT, 1, min_dist, 15, 10, 1, 5 );
    HoughCircles( image, circles, CV_HOUGH_GRADIENT, 1, min_dist, 15, 10, 1, 5);
    for (size_t i = 0; i< circles.size(); ++i) {
        KeyPoint tmp;
        tmp.pt.x = circles[i](0);
        tmp.pt.y = circles[i](1);
        tmp.size = circles[i](2);
        keypoints.push_back (tmp);
    }
}

int Pipeline2D::removeNonRemarquableCircles (std::vector<cv::KeyPoint> circles,
                                 std::vector<cv::KeyPoint> &remarquable_circles) {

}
