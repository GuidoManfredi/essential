#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "BoF.h"

class Pipeline2D {
  public:
    Pipeline2D ();

    void getGray(const cv::Mat& image, cv::Mat& gray);
    // Features functions
    bool detectFeatures(const cv::Mat& image,
						std::vector<cv::KeyPoint>& keypoints);
    bool describeFeatures(const cv::Mat image, std::vector<cv::KeyPoint> keypoints,
						  cv::Mat& descriptors);
    bool extractFeatures(const cv::Mat& image,
  						 std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
	bool match(const cv::Mat &desc1, const cv::Mat &desc2,
               std::vector<cv::DMatch>& matches);
    // BoW functions
    bool createVocabulary (std::vector<cv::Mat> images,
                           cv::Mat &vocabulary);
    /*bool createVocabulary (std::vector<cv::Mat> descriptors,
                           cv::Mat &vocabulary);*/
    void setVocabulary (cv::Mat vocabulary);
    cv::Mat getVocabulary();
    void saveVocabulary (std::string path);
    int loadVocabulary (std::string path);

    bool computeBoW (cv::Mat image, std::vector<cv::KeyPoint> keypoints,
                     cv::Mat &bow_descriptor);
    bool computeBoW (cv::Mat image, std::vector<cv::KeyPoint> keypoints,
                     cv::Mat &descriptors, cv::Mat &bow_descriptor);
    int matchBoW (const cv::Mat &query_bow, const std::vector<cv::Mat> &training_bow);
    double distanceBoW (cv::Mat query, cv::Mat train);
    // Bocabulary functions
    //bool computeVocabulary (cv::Mat descriptors, cv::Mat &vocabulary);
    void matchDescriptorsToVocabularies (const cv::Mat &query_descriptor,
                                         const std::vector<cv::Mat> &training_vocabulary,
                                         std::vector<int>& distances);
    void matchDescriptorsToVocabularies (const cv::Mat &query_descriptor,
                                         const std::vector<cv::Mat> &training_vocabulary,
                                         std::vector<double>& distances);
    void matchVocabularyToVocabulary (const cv::Mat &query_vocabulary, const std::vector<cv::Mat> &training_vocabulary,
                                      std::vector<double>& distances);

    unsigned int minNumberMatchesAllowed_;

  private:
    unsigned int number_visual_words_;

    cv::Ptr<cv::FeatureDetector>     detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher>   matcher_;

    cv::Ptr<cv::DescriptorMatcher>   bow_matcher_;
    cv::Ptr<cv::BOWKMeansTrainer> bow_trainer_;
    cv::Ptr<BoF> bow_extractor_;
};
