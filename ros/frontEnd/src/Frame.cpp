#include "Frame.h"

using namespace std;
using namespace cv;

//TODO reffine p3d_right with the results of solvePnPRansac

Frame::Frame () {
    number_matches_first_match_ = -1;
}

void Frame::setMatchesLeft (vector<DMatch> matches) {
	matches_left_.clear();
	p2d_left_.clear ();
	
	matches_left_ = matches;
	for (size_t i=0; i<matches.size (); ++i) {
		p2d_left_.push_back ( kpts_[matches[i].trainIdx].pt );
	}
}

void Frame::setMatchesRight (vector<DMatch> matches) {
	matches_right_.clear();
	p3d_right_.clear ();

	matches_right_ = matches;
	for (size_t i=0; i<matches.size (); ++i) {
		p3d_right_.push_back ( p3d_[matches[i].queryIdx] );
	}
	
	if ( number_matches_first_match_ == -1 )
	    number_matches_first_match_ = matches.size();
}

void Frame::filterNaNKeyPoints (Mat depth, vector<KeyPoint> kpts,
								vector<KeyPoint> &filtered_kpts,
								vector<Point3f> &filtered_p3d) {
	filtered_kpts.clear();
	filtered_p3d.clear();
	for (size_t i=0; i<kpts.size(); ++i) {
		Vec3f p3 = depth.at<Vec3f>(kpts[i].pt.y, kpts[i].pt.x); //* 1e3; // in meters, convert in milimeters
		if ( !isnan(p3[0]) && !isnan(p3[1]) && !isnan(p3[2]) ) {
			filtered_kpts.push_back (kpts[i]);
			filtered_p3d.push_back (p3); // looks like Vec3f ~= Point3f
		}
	}
}

void Frame::filterNaNKeyPoints (Mat depth, vector<KeyPoint> kpts,
								vector<int> &filtered_kpts_index,
								vector<Point3f> &filtered_p3d) {
	filtered_kpts_index.clear();
	filtered_p3d.clear();
	for (size_t i=0; i<kpts.size(); ++i) {
		Vec3f p3 = depth.at<Vec3f>(kpts[i].pt.y, kpts[i].pt.x); //* 1e3; // in meters, convert in milimeters
		if ( !isnan(p3[0]) && !isnan(p3[1]) && !isnan(p3[2]) ) {
			filtered_kpts_index.push_back (i);
			filtered_p3d.push_back (p3); // looks like Vec3f ~= Point3f
		}
	}
}
