#include "SkinSeg.h"

using namespace std;
using namespace cv;

SkinSeg::SkinSeg(int num_gaussians):num_gaussians_(num_gaussians) {
    model_ = EM(num_gaussians, EM::COV_MAT_SPHERICAL, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 300, 0.1));
}

void SkinSeg::train(string model_path) {
    cout << "Training...";
    Mat model = imread(model_path, CV_LOAD_IMAGE_COLOR);
    //Mat crop_model = model(Rect(40,40,160,160));
    //imshow("Model", crop_model); waitKey(0);

    //Mat crop_model_hsv;
    //cvtColor(crop_model, crop_model_hsv, CV_BGR2HSV);
    //train (crop_model_hsv);

    Mat model_hsv;
    cvtColor(model, model_hsv, CV_BGR2HSV);
    train (model_hsv);
    cout << "...done." << endl;
}

void SkinSeg::segment(cv::Mat img, cv::Mat &mask) {
    Mat hsv;
    cvtColor(img, hsv, CV_BGR2HSV);

    mask = Mat::zeros(img.size(), CV_8UC1);
    Mat samples;
    image2column(hsv, samples);

    vector<Mat> segmented;
    for( int i = 0; i < num_gaussians_; i++ )
        segmented.push_back( Mat::zeros( img.rows, img.cols, CV_8UC3 ) );

    if (model_.isTrained()) {
        Mat sample(1, 3, CV_32F);
        // classify every image pixel
        int idx = 0;
        for( int y = 0; y < img.rows; y++ ) {
            for( int x = 0; x < img.cols; x++ ) {
                //int response = cvRound(model_.predict(samples.row(idx++))[0]);
                int result = model_.predict(samples.row(idx++))[1];
                segmented[result].at<Point3i>(y, x, 0) = img.at<Point3i>(y, x, 0);
                if (result == 2) mask.at<char>(y, x) = 255.0;
                //cout << result << endl;
            }
        }
    } else {
        cout << "ExpectationModelisation not trained. Can't segment." << endl;
    }

/*
    for (size_t i = 0; i < num_gaussians_; ++i) {
        imshow("Debug", segmented[i]); waitKey(0);
    }
*/
    //mask = segmented[1];
    post_process(mask);
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
/*
void SkinSeg::train(std::vector<cv::Mat> source) {
    Mat samples;
    image2column(source, samples);

    Mat labels;
    model_.train(samples, noArray(), labels, noArray());
}
*/
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

void SkinSeg::post_process (cv::Mat &mask) {
    int erosion_type;
    //erosion_type = MORPH_RECT;
    //erosion_type = MORPH_CROSS;
    erosion_type = MORPH_ELLIPSE;
    int erosion_size = 2;

    int dilation_type;
    //dilation_type = MORPH_RECT;
    //dilation_type = MORPH_CROSS;
    dilation_type = MORPH_ELLIPSE;
    int dilation_size = 3;

    Mat erosion_el = getStructuringElement(erosion_type,
                                            Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                            Point( erosion_size, erosion_size ) );
    Mat dilation_el = getStructuringElement(dilation_type,
                                            Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                            Point( dilation_size, dilation_size ) );
    erode (mask,mask, erosion_el);
    erode (mask,mask, erosion_el);
    dilate (mask, mask, dilation_el);
}
