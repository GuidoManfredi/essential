#include "Pipeline2d.h"

Pipeline2d::Pipeline2d() {
  // init surf and sift modules.
	initModule_nonfree();

    _detector = new cv::SIFT();
    _extractor = new cv::SIFT();
    _matcher = new BFMatcher(cv::NORM_L2, false);

	_min_matches_allowed = 8;
}

void Pipeline2d::features ( const Mat img,
                            vector<KeyPoint> &kpts,	Mat &descs) {
    _detector->detect ( img, kpts );
    _extractor->compute ( img, kpts, descs );
}
