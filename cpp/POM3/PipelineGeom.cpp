#include <opencv2/calib3d/calib3d.hpp>

#include "PipelineGeom.h"

using namespace std;
using namespace cv;

PipelineGeom::PipelineGeom () {

}

void PipelineGeom::filterMatchesHomography (vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2,
                                            vector<DMatch> &matches, Mat &H) {
    std::vector<Point2f> pts1, pts2;
    match2points(keypoints1, keypoints2, matches, pts1, pts2);

    vector<unsigned char> mask(pts1.size());
    H = cv::findHomography(pts1, pts2, CV_RANSAC, 8.0, mask);
    //homography = cv::findHomography(pts1, pts2, CV_RANSAC, 0.5, mask);

    vector<DMatch> inliers;
    for (size_t i=0; i<mask.size(); i++) {
        if (mask[i])
            inliers.push_back(matches[i]);
    }
    matches.swap(inliers);
}

//void PipelineGeom::warpAndComputeHomography () {}

////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void PipelineGeom::match2points (vector<KeyPoint> kpts1, vector<KeyPoint> kpts2, vector<DMatch> matches,
                                 vector<Point2f> &pts1, vector<Point2f> &pts2) {
  for( int i = 0; i < matches.size(); i++ )
  {
    pts1.push_back( kpts1[matches[i].queryIdx ].pt );
    pts2.push_back( kpts2[matches[i].trainIdx ].pt );
  }
}
