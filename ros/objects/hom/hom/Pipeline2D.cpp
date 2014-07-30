#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "Pipeline2D.h"

using namespace std;
using namespace cv;

Pipeline2D::Pipeline2D() {
    detector_                                 = new cv::SIFT();
    extractor_                                = new cv::SIFT();
    matcher_                                  = new cv::BFMatcher(cv::NORM_L2, false);

    ratio_test_                               = true;
    minNumberMatchesAllowed_                  = 8;
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

vector<Point2f> Pipeline2D::getCorners(Mat image) {
    vector<Point2f> corners (4);
    corners[0] = Point2f (0, 0);
    corners[1] = Point2f (image.cols, 0);
    corners[2] = Point2f (image.cols, image.rows);
    corners[3] = Point2f (0, image.rows);
    return corners;
}
////////////////////////////////////////////////////////////////////////////////
// Features Part
////////////////////////////////////////////////////////////////////////////////
bool Pipeline2D::detectFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) {
    detector_->detect(image, keypoints);
    if (keypoints.empty())
        return false;
    return true;
}

bool Pipeline2D::describeFeatures(const cv::Mat image, std::vector<cv::KeyPoint> keypoints, cv::Mat& descriptors) {
    assert(!image.empty());
    assert(image.channels() == 1);

    extractor_->compute(image, keypoints, descriptors);
    if (keypoints.empty())
        return false;
    return true;
}

bool Pipeline2D::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    Mat gray;
    getGray(image, gray);
    detectFeatures(gray, keypoints);
    describeFeatures(gray, keypoints, descriptors);
    return true;
}

bool Pipeline2D::match (const cv::Mat &descs1, const cv::Mat &descs2,
                        std::vector<cv::DMatch>& matches) {
    matches.clear();
    if (ratio_test_) {
        // To avoid NaN's when best match has zero distance we will use inversed ratio.
        const float minRatio = 1.f / 1.5f;

        // KNN match will return 2 nearest matches for each query descriptor
        matcher_->knnMatch(descs1, descs2, knnMatches_, 2);

        for (size_t i=0; i<knnMatches_.size(); i++) {
            const cv::DMatch& bestMatch   = knnMatches_[i][0];
            const cv::DMatch& betterMatch = knnMatches_[i][1];

            float distanceRatio = bestMatch.distance / betterMatch.distance;

            // Pass only matches where distance ratio between
            // nearest matches is greater than 1.5 (distinct criteria)
            if (distanceRatio < minRatio) {
                matches.push_back(bestMatch);
            }
        }
    }
    else {
        // Perform regular match
        matcher_->match(descs1, descs2, matches);
    }
}
