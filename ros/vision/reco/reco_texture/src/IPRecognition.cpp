#include "IPRecognition.h"

using namespace std;
using namespace cv;

////////////////////////////////////////////////////////////////////////////////
//  PUBLIC METHODS
////////////////////////////////////////////////////////////////////////////////
IPRecognition::IPRecognition () {
  // init surf and sift modules.
	initModule_nonfree();

  //_detector = new cv::SIFT();
  //_extractor = new cv::SIFT();
  _detector = new cv::SURF();
  _extractor = new cv::SURF();
	_matcher = new BFMatcher(cv::NORM_L2, false);
}

IPRecognition::~IPRecognition () {
	;
}

void IPRecognition::load_model (string filepath) {
	Mat tmp_descs;
	
	FileStorage r_fs;
  r_fs.open (filepath, cv::FileStorage::READ);
  r_fs["descriptors"]>>tmp_descs;
}

void IPRecognition::recognize(const Mat img, const vector<string> candidates)
{
	vector<KeyPoint> query_kpts;
	Mat query_descs;
	vector<DMatch> matches;
	
	extract(img,  query_kpts, query_descs);
	int idx = 0;
	for (size_t i=0; i<candidates.size(); ++i) {
		if ((idx = name2idx(candidates[i])) != -1)
			match (query_descs, _train_descs[idx], matches);
		else
			cout << "Recognize : Warning : unknown candidates. Make sure all \
			models are trained with every type of visual cues." << endl;
	}
}

////////////////////////////////////////////////////////////////////////////////
//  PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void IPRecognition::extract(const Mat img, vector<KeyPoint> &kpts, Mat &descs) {
  _detector->detect ( img, kpts );
  _extractor->compute ( img, kpts, descs );
}

void IPRecognition::match(const Mat query, const Mat train,
													vector<DMatch>& matches)
{
  // QT : Query to Training
  vector< vector<DMatch> > sym_matches, matchesQT, matchesTQ;
  _matcher->knnMatch(query, train, matchesQT, 2);
  _matcher->knnMatch(train, query, matchesTQ, 2);

  symmetryCheck (matchesQT, matchesTQ, sym_matches);
  ratioCheck (sym_matches);

  // Keep only closest match
  for (size_t i=0; i<sym_matches.size (); ++i) {
    matches.push_back (sym_matches[i][0]);
  }
}

void IPRecognition::local_spatial_verification () {
;
}

void IPRecognition::add_descriptors (Mat in_descs) {  
  _train_descs.push_back (in_descs);
}

// For knn or radius matches
void IPRecognition::symmetryCheck (vector< vector<DMatch> > matches12,
																	 vector< vector<DMatch> > matches21,
			                             vector< vector<DMatch> > &filteredMatches) {
  for( size_t i = 0; i < matches12.size(); i++ ) {
      DMatch forward = matches12[i][0];
      DMatch backward = matches21[forward.trainIdx][0];
      if( backward.trainIdx == forward.queryIdx )
          filteredMatches.push_back( matches12[i] );
  }
}

void IPRecognition::ratioCheck(std::vector<std::vector<cv::DMatch> > &nmatches){
  // To avoid NaN's when best match has zero distance we will use inversed ratio.
  const float minRatio = 0.8f;//1.f / 1.5f;

  vector< vector<DMatch> > knmatches;
  for(size_t i=0; i<nmatches.size(); i++) {
    if((nmatches[i].size()==1)
    	||(nmatches[i][0].distance/nmatches[i][1].distance<minRatio))
      knmatches.push_back(nmatches[i]);
  }

  nmatches = knmatches;
}

int IPRecognition::name2idx (string name) {
	vector<string>::iterator it;
	if ((it = std::find(_names.begin(), _names.end(), name)) != _names.end()) {
		return _names.begin() - it;
	}
	return -1;
}
