#include "Matcher.h"

using namespace std;
using namespace cv;

Matcher::Matcher()
{
  // init surf and sift modules.
	initModule_nonfree();

  /*
  _detector = new cv::ORB(1000);
  _extractor = new cv::FREAK(false, false);
  _matcher = new BFMatcher(cv::NORM_HAMMING, false);
  */
  /*
  _detector = new FastFeatureDetector(TWEEK_DETECTOR);
  _extractor = new BriefDescriptorExtractor(TWEEK_DESCRIPTOR);//DescriptorExtractor::create("BRIEF");
	_matcher = new BFMatcher(cv::NORM_HAMMING, true);
	*/
	/*
  _detector = new cv::SIFT();
  _extractor = new cv::SIFT();
	_matcher = new BFMatcher(cv::NORM_L2, false);
	*/
    _detector = new cv::SURF();
    _extractor = new cv::SURF();
	_matcher = new BFMatcher(cv::NORM_L2, false);

	_min_number_matches_allowed = 8;
}

Matcher::~Matcher()
{}

void Matcher::extract ( const Mat img,
                        vector<KeyPoint> &kpts,	Mat &descs)
{
	// Detect keypoints
  _detector->detect ( img, kpts );
	// Extract descriptor for each keypoint
  _extractor->compute ( img, kpts, descs );
}

void Matcher::match(const Mat query_descs, vector<DMatch>& matches)
{
  // QT : Query to Training
  vector< vector<DMatch> > sym_matches, matchesQT, matchesTQ;
  _matcher->knnMatch(query_descs, _train_descs, matchesQT, 2);
  _matcher->knnMatch(_train_descs, query_descs, matchesTQ, 2);
  //cout << "Init matches " << matchesQT.size () << "/" << matchesTQ.size () << endl;

  symmetryCheck (matchesQT, matchesTQ, sym_matches);
  //cout << "Sym matches " << sym_matches.size () << endl;
  ratioCheck (sym_matches);
  //cout << "Ratio OK matches " << sym_matches.size () << endl;

  // Keep only closest match
  for (size_t i=0; i<sym_matches.size (); ++i) {
    matches.push_back (sym_matches[i][0]);
  }
}

// Already done in the bruteforce matcher when param 2 of constructor == true
void Matcher::symmetryCheck (vector<DMatch> matches12, vector<DMatch> matches21,
                   vector<DMatch> &filteredMatches)
{
  for( size_t i = 0; i < matches12.size(); i++ )
  {
      DMatch forward = matches12[i];
      DMatch backward = matches21[forward.trainIdx];
      if( backward.trainIdx == forward.queryIdx )
          filteredMatches.push_back( forward );
  }
}

// For knn or radius matches
void Matcher::symmetryCheck (vector< vector<DMatch> > matches12, vector< vector<DMatch> > matches21,
                              vector< vector<DMatch> > &filteredMatches)
{
  for( size_t i = 0; i < matches12.size(); i++ )
  {
      DMatch forward = matches12[i][0];
      DMatch backward = matches21[forward.trainIdx][0];
      if( backward.trainIdx == forward.queryIdx )
          filteredMatches.push_back( matches12[i] );
  }
}

void Matcher::ratioCheck (std::vector<std::vector<cv::DMatch> > &nmatches)
{
  // To avoid NaN's when best match has zero distance we will use inversed ratio.
  const float minRatio = 0.8f;//1.f / 1.5f;
  vector< vector<DMatch> > knmatches;
  for(int i=0; i<nmatches.size(); i++) {
    //cout << nmatches[i][0].distance/nmatches[i][1].distance << endl;
    if((nmatches[i].size()==1)||(nmatches[i][0].distance/nmatches[i][1].distance<minRatio)) {
      knmatches.push_back(nmatches[i]);
    }
  }
  nmatches.swap(knmatches);
}
