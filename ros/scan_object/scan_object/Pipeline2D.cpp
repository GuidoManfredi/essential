#include "Pipeline2D.h"

using namespace std;
using namespace cv;

Pipeline2D::Pipeline2D() {

    detector_                                 = new cv::SIFT();
    extractor_                                = new cv::SIFT();
    matcher_                                  = new cv::BFMatcher(cv::NORM_L2, true);
/*
  detector_                                 = FeatureDetector::create("FAST");
  extractor_                                = DescriptorExtractor::create("BRIEF");
  matcher_                                  = new cv::BFMatcher(cv::NORM_HAMMING, true);
*/
  minNumberMatchesAllowed_                  = 8;
  matches_kept_                             = 0.8;
}

void Pipeline2D::getGray(const cv::Mat& image, cv::Mat& gray)
{
    assert(!image.empty());
    if (image.channels()  == 3)
        cvtColor(image, gray, CV_BGR2GRAY);
    else if (image.channels() == 4)
        cvtColor(image, gray, CV_BGRA2GRAY);
    else if (image.channels() == 1)
        gray = image;
}

bool Pipeline2D::detectFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
    assert(!image.empty());
    assert(image.channels() == 1);

    detector_->detect(image, keypoints);
    if (keypoints.empty())
        return false;
    return true;
}

bool Pipeline2D::describeFeatures(const cv::Mat image, std::vector<cv::KeyPoint> keypoints, cv::Mat& descriptors)
{
    assert(!image.empty());
    assert(image.channels() == 1);

    if (!keypoints.empty()) {
	    extractor_->compute(image, keypoints, descriptors);
    	return true;
    }

    return false;
}

bool Pipeline2D::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    assert(!image.empty());
    assert(image.channels() == 1);

    detector_->detect(image, keypoints);
    if (keypoints.empty())
        return false;

    extractor_->compute(image, keypoints, descriptors);
    if (keypoints.empty())
        return false;
    //cout << "kpt " <<  keypoints.size() << endl;
    return true;
}

bool Pipeline2D::match (const cv::Mat &desc1, const cv::Mat &desc2,
                        vector<cv::DMatch>& matches)
{
  assert (desc1.data && desc2.data);
  matches.clear ();
  matcher_->match(desc1, desc2, matches);

  if (matches.size() > minNumberMatchesAllowed_)
    return true;

  return false;
}

void Pipeline2D::filterMatches (vector<DMatch> matches,
                                vector<DMatch> &filtered_matches) {
    filtered_matches.clear ();
    sort (matches.begin(), matches.end(), lessThanDistance());
    for (size_t i=0; i<matches.size()*matches_kept_; ++i) {
        filtered_matches.push_back (matches[i]);
    }
}
