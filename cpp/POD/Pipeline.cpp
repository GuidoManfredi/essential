#include <cmath>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <cassert>

#include "Pipeline.h"

Pipeline::Pipeline() {
  /*
        cv::Ptr<cv::FeatureDetector>     detector  = new cv::ORB(1000),
        cv::Ptr<cv::DescriptorExtractor> extractor = new cv::FREAK(false, false),
        cv::Ptr<cv::DescriptorMatcher>   matcher   = new cv::BFMatcher(cv::NORM_HAMMING, true),
        */
	m_detector                                 = new cv::SURF(400);
  m_extractor                                = new cv::SURF();
  m_matcher                                  = new cv::BFMatcher(cv::NORM_L2, true);

  enableRatioTest                            = false;
  enableHomographyRefinement                 = true;
  homographyReprojectionThreshold            = 3.0;
  minNumberMatchesAllowed                    = 8;
}

void Pipeline::getGray(const cv::Mat& image, cv::Mat& gray)
{
    if (image.channels()  == 3)
        cv::cvtColor(image, gray, CV_BGR2GRAY);
    else if (image.channels() == 4)
        cv::cvtColor(image, gray, CV_BGRA2GRAY);
    else if (image.channels() == 1)
        gray = image;
}

bool Pipeline::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const
{
    assert(!image.empty());
    assert(image.channels() == 1);

    m_detector->detect(image, keypoints);
    if (keypoints.empty())
        return false;

    m_extractor->compute(image, keypoints, descriptors);
    if (keypoints.empty())
        return false;

    return true;
}

bool Pipeline::gpu_extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const
{
    assert(!image.empty());
    assert(image.channels() == 1);

    cv::gpu::GpuMat img (image);
    cv::gpu::GpuMat keypoints1GPU, keypoints2GPU;
    cv::gpu::GpuMat descriptors1GPU, descriptors2GPU;
    m_detector->detect(image, keypoints);
    if (keypoints.empty())
        return false;

    m_extractor->compute(image, keypoints, descriptors);
    if (keypoints.empty())
        return false;

    return true;
}

void Pipeline::add_descriptors () {
;
}

void Pipeline::getMatches(const cv::Mat& queryDescriptors, const cv::Mat& objectDescriptors,
                           std::vector<cv::DMatch>& matches)
{
    matches.clear();

    if (enableRatioTest) {
        // To avoid NaN's when best match has zero distance we will use inversed ratio.
        const float minRatio = 1.f / 1.5f;

        // KNN match will return 2 nearest matches for each query descriptor
        m_matcher->knnMatch(queryDescriptors, objectDescriptors, m_knnMatches, 2);

        for (size_t i=0; i<m_knnMatches.size(); i++) {
            const cv::DMatch& bestMatch   = m_knnMatches[i][0];
            const cv::DMatch& betterMatch = m_knnMatches[i][1];

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
        m_matcher->match(queryDescriptors, objectDescriptors, matches);
    }
}

void Pipeline::getMatches(const cv::Mat& queryDescriptors,
                          std::vector<cv::DMatch>& matches)
{
    matches.clear();

    if (enableRatioTest) {
        // To avoid NaN's when best match has zero distance we will use inversed ratio.
        const float minRatio = 1.f / 1.5f;

        // KNN match will return 2 nearest matches for each query descriptor
        m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);

        for (size_t i=0; i<m_knnMatches.size(); i++) {
            const cv::DMatch& bestMatch   = m_knnMatches[i][0];
            const cv::DMatch& betterMatch = m_knnMatches[i][1];

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
        m_matcher->match(queryDescriptors, matches);
    }
}

bool Pipeline::refineMatchesWithHomography (
    const std::vector<cv::KeyPoint>& queryKeypoints,
    const std::vector<cv::KeyPoint>& trainKeypoints,
    float reprojectionThreshold,
    std::vector<cv::DMatch>& matches,
    cv::Mat& homography)
{
    if (matches.size() < minNumberMatchesAllowed)
        return false;

    // Prepare data for cv::findHomography
    std::vector<cv::Point2f> srcPoints(matches.size());
    std::vector<cv::Point2f> dstPoints(matches.size());
    for (size_t i = 0; i < matches.size(); i++) {
        srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
        dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
    }

    // Find homography matrix and get inliers mask
    std::vector<unsigned char> inliersMask(srcPoints.size());
    homography = cv::findHomography(srcPoints,
                                    dstPoints,
                                    CV_FM_RANSAC,
                                    reprojectionThreshold,
                                    inliersMask);

    std::vector<cv::DMatch> inliers;
    for (size_t i=0; i<inliersMask.size(); i++)
    {
        if (inliersMask[i])
            inliers.push_back(matches[i]);
    }

    matches.swap(inliers);
    return matches.size() > minNumberMatchesAllowed;
}

bool Pipeline::find (const cv::Mat& image, PlanarObject& object)
{
  // Convert input image to gray
  getGray(image, m_grayImg);
  // Extract feature points from input gray image
  extractFeatures(m_grayImg, m_queryKeypoints, m_queryDescriptors);
  // Get matches with current pattern
  getMatches(m_queryDescriptors, m_matches);
  // Find homography transformation and detect good matches
  bool homographyFound = refineMatchesWithHomography(
                                              m_queryKeypoints,
                                              object.keypoints,
                                              homographyReprojectionThreshold,
                                              m_matches,
                                              m_roughHomography);

  if (homographyFound) {
      // If homography refinement enabled improve found transformation
      if (enableHomographyRefinement) {
          // Warp image using found homography
          cv::warpPerspective(m_grayImg, m_warpedImg, m_roughHomography,
                              object.size, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
          // Get refined matches:
          std::vector<cv::KeyPoint> warpedKeypoints;
          std::vector<cv::DMatch> refinedMatches;

          // Detect features on warped image
          extractFeatures(m_warpedImg, warpedKeypoints, m_queryDescriptors);

          // Match with pattern
          getMatches(m_queryDescriptors, refinedMatches);

          // Estimate new refinement homography
          homographyFound = refineMatchesWithHomography(
                                              warpedKeypoints,
                                              object.keypoints,
                                              homographyReprojectionThreshold,
                                              refinedMatches,
                                              m_refinedHomography);

          // Get a result homography as result of matrix product of refined and rough homographies:
          object.homography = m_roughHomography * m_refinedHomography;

          // Transform contour with precise homography
          cv::perspectiveTransform(object.corners2d, object.points2d, object.homography);
      }
      else {
          object.homography = m_roughHomography;
          // Transform contour with rough homography
          cv::perspectiveTransform(object.corners2d, object.points2d, m_roughHomography);
      }
  }

  return homographyFound;
}
