#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "Pipeline2D.h"

using namespace std;
using namespace cv;

Pipeline2D::Pipeline2D() {
    // 500 = best
    number_visual_words_ = 500; // vocabulary size

    detector_                                 = new cv::SIFT();
    extractor_                                = new cv::SIFT();
    matcher_                                  = new cv::BFMatcher(cv::NORM_L2, true);

    bow_matcher_                              = new cv::BFMatcher(cv::NORM_L2, false);
    bow_trainer_                              = new BOWKMeansTrainer(number_visual_words_);
    bow_extractor_                            = new BoF(extractor_, bow_matcher_);

    minNumberMatchesAllowed_                  = 8;
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
////////////////////////////////////////////////////////////////////////////////
// Features Part
////////////////////////////////////////////////////////////////////////////////
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
    detectFeatures(image, keypoints);
    return describeFeatures(image, keypoints, descriptors);
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
////////////////////////////////////////////////////////////////////////////////
// BoW Part
////////////////////////////////////////////////////////////////////////////////

bool Pipeline2D::createVocabulary (vector<Mat> images,
                                   Mat &vocabulary) {
    vector<KeyPoint> tmp_kpts;
    cv::Mat tmp_descs;
    for ( size_t i = 0; i < images.size(); ++i ) {
        //imshow("kikou",images[i]); waitKey(0);
        Mat gray;
        getGray(images[i], gray);
        extractFeatures(gray, tmp_kpts, tmp_descs);
        bow_trainer_->add(tmp_descs);
    }
    vocabulary = bow_trainer_->cluster();
}
/*
bool Pipeline2D::createVocabulary (vector<Mat> descriptors,
                                   Mat &vocabulary) {
    for ( size_t i = 0; i < descriptors.size(); ++i ) {
        bow_trainer_->add(descriptors[i]);
    }
    vocabulary = bow_trainer_->cluster();
}
*/
bool Pipeline2D::computeBoW (Mat image, vector<KeyPoint> keypoints,
                             Mat &bow_descriptor) {
    bow_extractor_->compute (image, keypoints, bow_descriptor);
}

bool Pipeline2D::computeBoW (Mat image, vector<KeyPoint> keypoints,
                             Mat &descriptors, Mat &bow_descriptor) {
    vector<vector<int> > pointIdxsOfClusters;
    bow_extractor_->compute(image, keypoints, bow_descriptor, &pointIdxsOfClusters, &descriptors);
}
////////////////////////////////////////////////////////////////////////////////
// Vocabulary Part
////////////////////////////////////////////////////////////////////////////////
/*
bool Pipeline2D::computeVocabulary (cv::Mat descriptors, cv::Mat &vocabulary) {
    vector<Mat> descriptors_vec;
    descriptors_vec.push_back (descriptors);
    createVocabulary (descriptors_vec, vocabulary);
}
*/
void Pipeline2D::setVocabulary (cv::Mat vocabulary) {
    bow_extractor_->setVocabulary(vocabulary);
}

cv::Mat Pipeline2D::getVocabulary() {
    return bow_extractor_->getVocabulary();
}

void Pipeline2D::saveVocabulary (string path) {
    cv::FileStorage fs(path.c_str(), cv::FileStorage::WRITE);
    fs << "Vocabulary" << bow_extractor_->getVocabulary();
    fs.release();
}

int Pipeline2D::loadVocabulary (string path) {
    cv::FileStorage fs(path.c_str(), cv::FileStorage::READ);
    Mat vocabulary;
    fs["Vocabulary"] >> vocabulary;
    setVocabulary(vocabulary);
    fs.release();
    return vocabulary.rows;
}

