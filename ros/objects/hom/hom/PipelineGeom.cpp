#include "PipelineGeom.h"

using namespace std;
using namespace cv;

PipelineGeom::PipelineGeom() {}

bool PipelineGeom::computeHomography (std::vector<cv::KeyPoint> kpts1, std::vector<cv::KeyPoint> kpts2,
                                        std::vector<cv::DMatch>& matches,
                                        cv::Mat &H) {
    const int minNumberMatchesAllowed = 8;
    float reproj_thresh = 3.0;

    if (matches.size() < minNumberMatchesAllowed)
        return false;

    // Prepare data for cv::findHomography
    vector<Point2f> points1, points2;
    keypoints2points(kpts1, kpts2, matches, points1, points2);

    // Find homography matrix and get inliers mask
    std::vector<unsigned char> inliers_mask(matches.size());
    H = cv::findHomography(points1, points2,
                           CV_RANSAC, reproj_thresh,
                           inliers_mask);

    std::vector<cv::DMatch> inliers;
    for (size_t i=0; i<inliers_mask.size(); i++) {
        if (inliers_mask[i])
            inliers.push_back(matches[i]);
    }

    matches.swap(inliers);
    return matches.size() > minNumberMatchesAllowed;
}

bool PipelineGeom::computePose(Face frame, Face face,
                                  Mat &P, std::vector<cv::DMatch> &inliers) {
    Mat raw_H, H;
    cout << "Refined matches from " << inliers.size();
    bool found = computeHomography (frame.keypoints_, face.keypoints_,
                                     inliers, raw_H);
    cout << " to " << inliers.size() << endl;
    if (found) {
        cout << "Found candidate homography. Further refining." << endl;
        Face warped;
        //cv::warpPerspective(frame.image_, warped.image_, raw_H, face.size_, cv::WARP_INVERSE_MAP |cv::INTER_CUBIC);
        cv::warpPerspective(frame.image_, warped.image_, raw_H, face.size_, cv::INTER_CUBIC);
        //imshow("Debug", warped.image_); waitKey(0);
        pip_.extractFeatures(warped.image_, warped.keypoints_, warped.descriptors_);

        std::vector<cv::DMatch> refined_matches;
        pip_.match(warped.descriptors_, face.descriptors_, refined_matches);

        Mat refined_H;
        found = computeHomography(warped.keypoints_, face.keypoints_,
                                  refined_matches, refined_H);

        if (found)
            H = raw_H * refined_H;
        else
            H = raw_H;

// DEBUG
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = Point2f(0,0); obj_corners[1] = Point2f( face.size_.width, 0 );
        obj_corners[2] = Point2f( face.size_.width, face.size_.height); obj_corners[3] = Point2f( 0, face.size_.height );
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform( obj_corners, scene_corners, H.inv());
        //perspectiveTransform( obj_corners, scene_corners, H);
        // draw
        Mat out;
        frame.image_.copyTo(out);
        line( out, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4 );
        line( out, scene_corners[1], scene_corners[2], Scalar( 0, 255, 0), 4 );
        line( out, scene_corners[2], scene_corners[3], Scalar( 0, 255, 0), 4 );
        line( out, scene_corners[3], scene_corners[0], Scalar( 0, 255, 0), 4 );
        imshow("Debug", out); waitKey(1);
//
        homography2transform(H, P);
    }
    return found;
}
////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHOD
////////////////////////////////////////////////////////////////////////////////
void PipelineGeom::keypoints2points(vector<KeyPoint> kpts1, vector<KeyPoint> kpts2, vector<DMatch> matches,
                                     vector<Point2f> &points1, vector<Point2f> &points2) {
    points1.resize(matches.size()); points2.resize(matches.size());

    for (size_t i = 0; i < matches.size(); i++) {
        points1[i] = kpts1[matches[i].queryIdx].pt;
        points2[i] = kpts2[matches[i].trainIdx].pt;
    }
}

void PipelineGeom::keypoints2points(vector<KeyPoint> kpts, vector<Point2f> &points) {
    points.resize(kpts.size());
    for (size_t i = 0; i < kpts.size(); i++) {
        points[i] = kpts[i].pt;
    }
}

void PipelineGeom::points2keypoints(vector<Point2f> points, vector<KeyPoint> &kpts) {
    kpts.resize(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        kpts[i].pt = points[i];
    }
}

void PipelineGeom::homography2transform(Mat H, Mat &P) {
    P = (Mat_<double>(4,4) <<
         H.at<double>(0,0), H.at<double>(0,1), 0, H.at<double>(0,2),
         H.at<double>(1,0), H.at<double>(1,1), 0, H.at<double>(1,2),
         H.at<double>(2,0), H.at<double>(2,1), 0, H.at<double>(2,2),
         0,                 0,                0, 1);
}
