#include "Detector.h"

using namespace cv;
using namespace std;

Detector::Detector (string cascade_name):cascade_name_(cascade_name) {
	if (!cascade_.load (cascade_name_))
		cout << "Error: Detector: Could not load classifier cascade." << endl;
}

vector<Rect> Detector::detect (Mat img) {
	vector<Rect> hands;
	int scale = 1;
	Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );
	cvtColor( img, gray, CV_BGR2GRAY );
  resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
  equalizeHist( smallImg, smallImg );
  
  double scaleFactor = 1.1;
  int min_neighbors = 10;
  cascade_.detectMultiScale( smallImg, hands, scaleFactor, min_neighbors, 0
    |CV_HAAR_FIND_BIGGEST_OBJECT
    //|CV_HAAR_DO_ROUGH_SEARCH
    |CV_HAAR_SCALE_IMAGE
    , Size(30, 30) );
  //cout << hands.size () << " hands." << endl;
  // not sure this is usefull, seems to always have aspect ratio = 1
  for (size_t i=0; i<hands.size(); ++i) {
	  double aspect_ratio = (double)hands[i].width/hands[i].height;
		if( aspect_ratio < 0.75 || 1.3 < aspect_ratio )
			hands.erase(hands.begin()+i);
  }
  
  return hands;
}

Rect Detector::detect (Mat img, Rect loc) {

}
