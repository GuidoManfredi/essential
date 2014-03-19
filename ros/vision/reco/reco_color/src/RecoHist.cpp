#include "RecoHist.h"

using namespace std;
using namespace cv;
////////////////////////////////////////////////////////////////////////////////
//	PUBLIC METHODS
////////////////////////////////////////////////////////////////////////////////
RecoHist::RecoHist () {
	_compare_method = 1;
}

vector<string> RecoHist::recognize (Mat img, double thresh,
																		vector<string> candidates) {
	Mat hsv;
	cvtColor( img, hsv, CV_BGR2HSV );
  /// Using 10 bins for hue and 12 for saturation
  int h_bins = 10; int s_bins = 10;
  int histSize[] = { h_bins, s_bins };
  // hue varies from 0 to 256, saturation from 0 to 180
  float h_ranges[] = { 0, 256 };
  float s_ranges[] = { 0, 180 };
  const float* ranges[] = { h_ranges, s_ranges };
  // Use the 0-th and 1-st channels
  int channels[] = { 0, 1 };
  
  Mat hist;
  calcHist( &hsv, 1, channels, Mat(), hist, 2, histSize, ranges, true, false );
  normalize( hist, hist, 0, 1, NORM_MINMAX, -1, Mat() );
  
  return recognize (hist, thresh, candidates);
}

void RecoHist::load_training_file (string filepath) {

}

////////////////////////////////////////////////////////////////////////////////
//	PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
vector<string> RecoHist::recognize_hist (Mat hist, double thresh,
																				 vector<string> candidates) {
	int n = _vec_hist.size();
	vector<string> remaining_candidates;
	
	for (int i=0; i<n; ++i) {
		double dist = distance (hist, _vec_hist[i]);
		if (dist <= thresh &&
				find(candidates.begin(), candidates.end(), _vec_names[i]) != candidates.end())
			remaining_candidates.push_back (_vec_names[i]);
	}
	
	return candidates;
}

double RecoHist::distance (Mat hist1, Mat hist2) {
	return compareHist (hist1, hist2, _compare_method);
}


