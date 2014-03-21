#include "commons.h"

using namespace std;
using namespace cv;

Mat rt2P (Mat rvec, Mat tvec) {
    Mat R;
	Rodrigues(rvec, R);
    Mat P = (Mat_<float>(4, 4, CV_32F) <<	R.at<float>(0,0),	R.at<float>(0,1),	R.at<float>(0,2), tvec.at<float>(0),
                                 			R.at<float>(1,0),	R.at<float>(1,1),	R.at<float>(1,2), tvec.at<float>(1),
			                                R.at<float>(2,0),	R.at<float>(2,1),	R.at<float>(2,2), tvec.at<float>(2),
                            			    0                ,                  0,                  0,                  1);
    return P;
}

Mat Rt2P (Mat R, Mat tvec) {
	Mat P = (Mat_<float>(4, 4, CV_32F) <<	R.at<float>(0,0),	R.at<float>(0,1),	R.at<float>(0,2), tvec.at<float>(0),
                                 			R.at<float>(1,0),	R.at<float>(1,1),	R.at<float>(1,2), tvec.at<float>(1),
			                                R.at<float>(2,0),	R.at<float>(2,1),	R.at<float>(2,2), tvec.at<float>(2),
                            			    0                ,                  0,                  0,                  1);
    return P;
}
