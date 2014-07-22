#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

class BoF
{
  public:
    BoF( const cv::Ptr<cv::DescriptorExtractor>& dextractor,
         const cv::Ptr<cv::DescriptorMatcher>& dmatcher );
    virtual ~BoF();

    void setVocabulary( const cv::Mat& vocabulary );
    const cv::Mat& getVocabulary() const;
    void compute( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& imgDescriptor,
                  std::vector<std::vector<int> >* pointIdxsOfClusters=0, cv::Mat* descriptors=0 );

    int descriptorSize() const;
    int descriptorType() const;

  private:
    cv::Mat vocabulary;
    cv::Ptr<cv::DescriptorExtractor> dextractor;
    cv::Ptr<cv::DescriptorMatcher> dmatcher;
};
