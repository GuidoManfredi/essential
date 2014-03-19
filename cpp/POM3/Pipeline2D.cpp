#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "Pipeline2D.h"

using namespace std;
using namespace cv;

Pipeline2D::Pipeline2D() {
    number_visual_words_ = 100; // vocabulary size

    detector_                                 = new cv::SIFT();
    extractor_                                = new cv::SIFT();
    matcher_                                  = new cv::BFMatcher(cv::NORM_L2, true);

    bow_matcher_                              = new cv::BFMatcher(cv::NORM_L2, false);
    bow_trainer_ = new BOWKMeansTrainer(number_visual_words_);
    bow_extractor_ = new MyBOWImgDescriptorExtractor(extractor_, bow_matcher_);

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
bool Pipeline2D::createVocabulary (vector<Mat> descriptors,
                                   Mat &vocabulary) {
    for ( size_t i = 0; i < descriptors.size(); ++i ) {
        bow_trainer_->add(descriptors[i]);
    }
    vocabulary = bow_trainer_->cluster();
}

bool Pipeline2D::computeBoW (Mat image, vector<KeyPoint> keypoints,
                             Mat &bow_descriptor) {
    bow_extractor_->compute (image, keypoints, bow_descriptor);
}

bool Pipeline2D::computeBoW (Mat image, vector<KeyPoint> keypoints,
                             Mat &descriptors, Mat &bow_descriptor) {
    vector<vector<int> > pointIdxsOfClusters;
    bow_extractor_->compute(image, keypoints, bow_descriptor, &pointIdxsOfClusters, &descriptors);
}

bool Pipeline2D::computeBoW (Mat descriptors, Mat &bow_descriptor) {
    bow_extractor_->compute(descriptors, bow_descriptor);
}

int Pipeline2D::matchBoW (const cv::Mat &query_bow, const std::vector<cv::Mat> &training_bow) {
    double min_distance = 1000;
    int min_i = 0;
    for ( size_t i = 0; i < training_bow.size(); ++i ) {
        double distance = distanceBoW (query_bow, training_bow[i]);
        if (distance < min_distance) {
            min_i = i;
            min_distance = distance;
        }
    }
    return min_i;
}

double Pipeline2D::distanceBoW (Mat query, Mat train) {
    double distance = compareHist(query, train, CV_COMP_BHATTACHARYYA);
    return distance;
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

void Pipeline2D::matchDescriptorsToVocabularies (const cv::Mat &query_descriptor, const std::vector<cv::Mat> &training_vocabulary,
                                                 std::vector<int>& distances) {
    distances.clear();
    for (size_t i = 0; i < training_vocabulary.size(); ++i ) {
        std::vector<cv::DMatch> matches;
        matcher_->match (query_descriptor, training_vocabulary[i], matches);
        distances.push_back (matches.size());
    }
}

void Pipeline2D::matchDescriptorsToVocabularies (const cv::Mat &query_descriptor, const std::vector<cv::Mat> &training_vocabulary,
                                                 std::vector<double>& distances) {
    distances.clear();
    for (size_t i = 0; i < training_vocabulary.size(); ++i ) {
        std::vector<cv::DMatch> matches;
        matcher_->match (query_descriptor, training_vocabulary[i], matches);
        double tmp_distance = 0;
        for (size_t k = 0; k < matches.size(); ++k) {
            tmp_distance += matches[k].distance;
        }
        distances.push_back (tmp_distance);
    }
}

void Pipeline2D::matchVocabularyToVocabulary (const cv::Mat &query_vocabulary, const std::vector<cv::Mat> &training_vocabulary,
                                    std::vector<double>& distances) {
    distances.clear();
    for (size_t i = 0; i < training_vocabulary.size(); ++i ) {
        double distance = norm (query_vocabulary, training_vocabulary, CV_L2);
        distances.push_back (distance);
    }
}
