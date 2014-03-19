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

void Matcher::set_descriptors (vector<Mat> in_descs)
{
  // in vconcat source can't be empty, we initialize it
  _train_descs = in_descs[0];
  for (size_t i=1; i<in_descs.size (); ++i)
  {
    if ( !in_descs[i].empty() )
      vconcat(_train_descs, in_descs[i], _train_descs);
  }
}

void Matcher::add_descriptors (vector<Mat> in_descs)
{
  if (!_train_descs.empty())
  {
    for (size_t i=0; i<in_descs.size (); ++i)
    {
      if (!in_descs[i].empty())
        vconcat(_train_descs, in_descs[i], _train_descs);
    }
  }
  else
    set_descriptors (in_descs);
}

void Matcher::add_kpts_descriptors (vector< vector<KeyPoint> > in_kpts, vector<Mat> in_descs)
{
  // in vconcat source can't be empty, we initialize it
  _train_descs = in_descs[0];
  for (size_t k=0; k<in_kpts[0].size (); ++k)
    _train_kpts.push_back (in_kpts[0][k]);
  for (size_t i=1; i<in_descs.size (); ++i)
  {
    if ( !in_descs[i].empty () )
      vconcat(_train_descs, in_descs[i], _train_descs);
    for (size_t k=0; k<in_kpts[i].size (); ++k)
      _train_kpts.push_back (in_kpts[i][k]);
  }
  //cout << _train_descs.size () << endl;
}

void Matcher::match(const Mat query, vector<DMatch>& in_matches)
{
  // QT : Query to Training
  vector< vector<DMatch> > sym_matches, matchesQT, matchesTQ;
  _matcher->knnMatch(query, _train_descs, matchesQT, 2);
  _matcher->knnMatch(_train_descs, query, matchesTQ, 2);
  //cout << "Init matches " << matchesQT.size () << "/" << matchesTQ.size () << endl;

  symmetryCheck (matchesQT, matchesTQ, sym_matches);
  //cout << "Sym matches " << sym_matches.size () << endl;
  ratioCheck (sym_matches);
  //cout << "Ratio OK matches " << sym_matches.size () << endl;

  // Keep only closest match
  for (size_t i=0; i<sym_matches.size (); ++i)
  {
    in_matches.push_back (sym_matches[i][0]);
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
  for(int i=0; i<nmatches.size(); i++)
  {
    //cout << nmatches[i][0].distance/nmatches[i][1].distance << endl;
    if((nmatches[i].size()==1)||(nmatches[i][0].distance/nmatches[i][1].distance<minRatio))
    {
      knmatches.push_back(nmatches[i]);
    }
  }

  nmatches = knmatches;
}

Mat Matcher::get_image_matches (const Mat& img1, const Mat& img2,
																vector<KeyPoint> kpts1, vector<int> idx1,
																vector<KeyPoint> kpts2, vector<int> idx2)
{
	Mat outImg;
	int n = idx1.size();
	std::vector<cv::DMatch>	matches;
	for (size_t i=0; i<n; ++i)
	{
    matches.push_back ( DMatch(idx1[i], idx2[i], 1.0));
	}

	drawMatches ( img1, kpts1, img2, kpts2, matches, outImg,
								Scalar::all(-1), Scalar::all(-1), vector<char>(),
								DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	return outImg;
}

void Matcher::draw_matches (const Mat& img1, const Mat& img2,
														vector<KeyPoint> kpts1, vector<KeyPoint> kpts2,
														vector<DMatch> matches)
{
	Mat outImg;
	namedWindow ("Matches Debug");
	drawMatches ( img1, kpts1, img2, kpts2, matches, outImg,
								Scalar::all(-1), Scalar::all(-1), vector<char>(),
								DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imshow ("Matches Debug", outImg);
	waitKey (0);
}

void Matcher::draw_matches (const Mat& img1, const Mat& img2,
														vector<KeyPoint> kpts,
														vector<DMatch> matches)
{
	Mat outImg;
	namedWindow ("Matches Debug");
	drawMatches ( img1, kpts, img2, _train_kpts, matches, outImg,
								Scalar::all(-1), Scalar::all(-1), vector<char>(),
								DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imshow ("Matches Debug", outImg);
	waitKey (0);
}

void Matcher::draw_kpts (const Mat& img, vector<KeyPoint> kpts)
{
	Mat outImg;
	outImg = img;
	namedWindow ("Key Points Debug");
  for (size_t i=0; i<kpts.size(); ++i)
     circle(outImg, kpts[i].pt, 3.0, Scalar(0, 0, 255));
	imshow ("Key Points Debug", outImg);
	waitKey (0);
}

