#include "SkinSeg.h"

using namespace std;
using namespace cv;

SkinSeg::SkinSeg(int num_gaussians) {
    model_ = EM(num_gaussians, EM::COV_MAT_SPHERICAL, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 300, 0.1));
}

void SkinSeg::train(string model_path) {
    Mat model = imread(model_path, CV_LOAD_IMAGE_COLOR);
    train (model);
}

void SkinSeg::segment(cv::Mat img, cv::Mat &mask) {
    mask = Mat::zeros(img.size(), CV_32S);
    Mat samples;
    image2column(img, samples);

    if (model_.isTrained()) {
        Mat sample(1, 3, CV_32F);
        // classify every image pixel
        int idx = 0;
        for( int i = 0; i < img.rows; i++ ) {
            for( int j = 0; j < img.cols; j++ ) {
                int response = cvRound(model_.predict(samples.row(idx++))[0]);
                //mask.at<int>(i, j) = response*(-10);
                //cout << response << endl;
            }
        }
    } else {
        cout << "ExpectationModelisation not trained. Can't segment." << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void SkinSeg::train(cv::Mat source) {
    Mat samples;
    image2column(source, samples);

    Mat labels;
    model_.train(samples, noArray(), labels, noArray());
}

void SkinSeg::image2column(cv::Mat source, cv::Mat &samples) {
    samples = Mat(source.rows * source.cols, 3, CV_32FC1);
    //convert the input image to float
    cv::Mat floatSource;
    source.convertTo(floatSource, CV_32F);
    //now convert the float image to column vector
    int idx = 0;
    for (int y = 0; y < source.rows; y++) {
        cv::Vec3f* row = floatSource.ptr<cv::Vec3f > (y);
        for (int x = 0; x < source.cols; x++) {
            samples.at<cv::Vec3f > (idx++, 0) = row[x];
        }
    }
}

