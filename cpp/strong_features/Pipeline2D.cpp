#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "Pipeline2D.h"

using namespace std;
using namespace cv;

Pipeline2D::Pipeline2D(Mat K): K_(K) {
    // init surf and sift modules.
    initModule_nonfree();
    //setFeatures(eSIFT);
    setFeatures(eASIFT);
}

void Pipeline2D::setFeatures (Feature ft) {
    ASIFT_ = false;
    if (ft == eASIFT) {
        ASIFT_ = true;
        matcher_ = new BFMatcher(cv::NORM_L2, true);
    } else if (ft == eSIFT) {
        cout << "Using SIFTs. " << endl;
        detector_ = new cv::SIFT();
        extractor_ = new cv::SIFT();
        matcher_ = new BFMatcher(cv::NORM_L2, true);
    } else if (ft == eSURF) {
        cout << "Using SURFs. " << endl;
        detector_ = new cv::SURF();
        extractor_ = new cv::SURF();
        matcher_ = new BFMatcher(cv::NORM_L2, true);
    } else if (ft == eFREAK) {
        cout << "Using FREAKs. " << endl;
        detector_ = new cv::ORB(1000);
        extractor_ = new cv::FREAK(false, false);
        matcher_ = new BFMatcher(cv::NORM_HAMMING, true);
    } else if (ft == eBRIEF) {
        cout << "Using BRIEFs. " << endl;
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
        cout << "Using ASIFT" << endl;
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

int Pipeline2D::match (const cv::Mat &descs1, const cv::Mat &descs2, vector<DMatch> &matches) {
    matcher_->match(descs1, descs2, matches);
    return matches.size();
}

float Pipeline2D::estimate_pose (vector<KeyPoint> keypoints, vector<Point3f> points,
                                  vector<DMatch> &matches) {
    vector<Point2f> kpts;
    vector<Point3f> pts;
    get_matched_point(keypoints, points, matches,
                      kpts, pts);
    Mat rvec = Mat::zeros(1, 3, CV_32F);
    Mat tvec = Mat::zeros(1, 3, CV_32F);
    //K_.convertTo (K_, CV_32F);
    cout << "Estimating pose with " << pts.size() << " points." << endl;
    vector<int> inliers;
    /*
    solvePnPRansac (Mat(pts), Mat(kpts), K_, Mat(), rvec, tvec,
                    false, 500, 4.0, 0.9 * kpts.size(), inliers, CV_EPNP);
    */
    solvePnPRansac (Mat(pts), Mat(kpts), K_, Mat(), rvec, tvec, false, CV_EPNP);
    solvePnP (Mat(pts), Mat(kpts), K_, Mat(), rvec, tvec, true);
    rvec.convertTo (rvec, CV_32F);
    tvec.convertTo (tvec, CV_32F);
    /*
    cout << rvec.at<float>(0) * 180 / M_PI << " "
         << rvec.at<float>(1) * 180 / M_PI << " "
         << rvec.at<float>(2) * 180 / M_PI << endl;
    */
    return rvec.at<float>(1);
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

void Pipeline2D::get_matched_point(vector<KeyPoint> keypoints, vector<Point3f> points, vector<DMatch> matches,
                        vector<Point2f> &kpts, vector<Point3f> &pts) {
    int n = matches.size();
    kpts.clear();
    pts.clear();
    for (size_t i = 0; i < n; ++i) {
        Point3f pt = points[matches[i].trainIdx];
        //Point3f pt = points[matches[i].queryIdx];
        if ( pt.x != 0.0 || pt.y != 0.0 || pt.z != 0.0 ) { // remove (0.0, 0.0, 0.0) points
            kpts.push_back(keypoints[matches[i].queryIdx].pt);
            //kpts.push_back(keypoints[matches[i].trainIdx].pt);
            pts.push_back(pt);
        }
    }
}

float Pipeline2D::dist_angle (float a, float b) {
    if ( a >= 180 && b < 180 )
        return fabs(360 - a + b);
    else if ( a < 180 && b >= 180 )
        return fabs(a + 360 - b);
    else if ( a >= 180 && b >= 180 )
        return fabs(a - b);
    else if ( a < 180 && b < 180 )
        return fabs(a - b);
}
