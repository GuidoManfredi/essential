#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "Pipeline2D.h"

using namespace std;
using namespace cv;

Pipeline2D::Pipeline2D() {
    // init surf and sift modules.
    initModule_nonfree();
    ASIFT_ = false;
    //setFeatures(eSIFT);
    setFeatures(eASIFT);
}

void Pipeline2D::setFeatures (Feature ft) {
    if (ft == eASIFT) {
        ASIFT_ = true;
        matcher_ = new BFMatcher(cv::NORM_L2, true);
    } else if (ft == eSIFT) {
        detector_ = new cv::SIFT();
        extractor_ = new cv::SIFT();
        matcher_ = new BFMatcher(cv::NORM_L2, true);
    } else if (ft == eSURF) {
        detector_ = new cv::SURF();
        extractor_ = new cv::SURF();
        matcher_ = new BFMatcher(cv::NORM_L2, true);
    } else if (ft == eFREAK) {
        detector_ = new cv::ORB(1000);
        extractor_ = new cv::FREAK(false, false);
        matcher_ = new BFMatcher(cv::NORM_HAMMING, true);
    } else if (ft == eBRIEF) {
        detector_ = new FastFeatureDetector();
        extractor_ = new BriefDescriptorExtractor();
        matcher_ = new BFMatcher(cv::NORM_HAMMING, true);
    } else {
        cout << "Error : setFeatures : Couldn't find specified feature" << endl;
    }
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

void Pipeline2D::extractDescriptors(const cv::Mat& image, const cv::Mat& mask,
                                    vector<KeyPoint> &keypoints, Mat &descriptors) {
    Mat gray;
    getGray(image, gray);
    Mat gray_masked;
    gray.copyTo(gray_masked, mask);
    // Detect keypoints
    if (ASIFT_) {
        vector<float> asift_image (gray_masked.data, gray_masked.data + gray_masked.cols * gray_masked.rows);
        int num_tilts = 7;
        siftPar siftparameters;
        default_sift_parameters(siftparameters);
        vector<vector<keypointslist > > keys;
        compute_asift_keypoints(asift_image, gray_masked.cols, gray_masked.rows, num_tilts, 0, keys, siftparameters);
        key2kpts(keys, keypoints);
        key2desc(keys, descriptors);
    } else {
        detector_->detect (gray_masked, keypoints, mask);
        extractor_->compute (gray_masked, keypoints, descriptors);
    }
}

int Pipeline2D::match (const cv::Mat &descs1, const cv::Mat &descs2) {
    std::vector<cv::DMatch> matches;
    matcher_->match(descs1, descs2, matches);
    return matches.size();
}
/*
int Pipeline2D::match (vector<vector<keypointslist > > keys1, vector<vector<keypointslist > > keys2,
                       int width1, int height1, int width2, int height2) {
    int num_tilts = 7;
    matchingslist matchings;
	siftPar siftparameters;
	default_sift_parameters(siftparameters);
    int num_matches = compute_asift_matches(num_tilts, num_tilts, width1, height1, width2,
                                             height2, 0, keys1, keys2, matchings, siftparameters);
    return num_matches;
}
*/
////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void Pipeline2D::key2desc (vector<vector<keypointslist > > key, Mat &desc) {
    for (size_t i = 0; i < key.size(); ++i) {
        for (size_t j = 0; j < key[i].size(); ++j) {
            for (size_t k = 0; k < key[i][j].size(); ++k) {
                Mat sift = Mat(1, 128, CV_32F, &key[i][j][k].vec);
                desc.push_back (sift);
            }
        }
    }
}

void Pipeline2D::key2kpts (vector<vector<keypointslist > > key, vector<KeyPoint> &kpts) {
    for (size_t i = 0; i < key.size(); ++i) {
        for (size_t j = 0; j < key[i].size(); ++j) {
            for (size_t k = 0; k < key[i][j].size(); ++k) {
                KeyPoint kpt;
                kpt.pt.x = key[i][j][k].x;
                kpt.pt.y = key[i][j][k].y;
                kpt.angle = key[i][j][k].angle;
                kpt.size = key[i][j][k].scale;
                kpts.push_back (kpt);
            }
        }
    }
}
