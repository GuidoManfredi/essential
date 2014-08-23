#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "Pipeline2D.h"

using namespace std;
using namespace cv;

Pipeline2D::Pipeline2D() {
    // init surf and sift modules.
    initModule_nonfree();

    /*
    detector_ = new cv::ORB(1000);
    extractor_ = new cv::FREAK(false, false);
    matcher_ = new BFMatcher(cv::NORM_HAMMING, false);
    */
    /*
    detector_ = new FastFeatureDetector();
    extractor_ = new BriefDescriptorExtractor();//DescriptorExtractor::create("BRIEF");
    matcher_ = new BFMatcher(cv::NORM_HAMMING, true);
    */

    detector_ = new cv::SIFT();
    extractor_ = new cv::SIFT();
    matcher_ = new BFMatcher(cv::NORM_L2, true);

    /*
    detector_ = new cv::SURF();
    extractor_ = new cv::SURF();
    matcher_ = new BFMatcher(cv::NORM_L2, true);
    */
}

void Pipeline2D::getGray(const cv::Mat& image, cv::Mat& gray) {
    assert(!image.empty());
    if (image.channels()  == 3)
        cvtColor(image, gray, CV_BGR2GRAY);
    else if (image.channels() == 4)
        cvtColor(image, gray, CV_BGRA2GRAY);
    else if (image.channels() == 1)
        gray = image;
}

void Pipeline2D::extractDescriptors(const cv::Mat& image, const cv::Mat& mask, Mat &descriptors) {
    Mat gray;
    getGray(image, gray);
    // Detect keypoints
    vector<KeyPoint> kpts;
    detector_->detect (gray, kpts, mask);
    // Extract descriptor for each keypoint
    extractor_->compute (gray, kpts, descriptors);
}

int Pipeline2D::match (const cv::Mat &descs1, const cv::Mat &descs2) {
    std::vector<cv::DMatch> matches;
    matcher_->match(descs1, descs2, matches);
    return matches.size();
}
