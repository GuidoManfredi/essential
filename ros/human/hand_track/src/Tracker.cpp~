#include "Tracker.h"

using namespace cv;
using namespace std;
////////////////////////////////////////////////////////////////////////////////
// PUBLIC METHODS
////////////////////////////////////////////////////////////////////////////////
Tracker::Tracker () {
	//hue_size_ = 100;
	//sat_size_ = 100;
	hue_size_ = 36;
	sat_size_ = 50;
	//val_min_ = 30;
	//val_max_ = 226;
	val_min_ = 60;
	val_max_ = 180;

  hue_range_[0] = 0;
  hue_range_[1] = 180;
  sat_range_[0] = 0;
  sat_range_[1] = 255;
  pranges_[0] = hue_range_;
  pranges_[1] = sat_range_;
	
	tracker_initialized = false;
}

RotatedRect Tracker::camshift_init (Mat img, Rect detection) {
	Mat hsv, backproj, mask;
	const int hist_size[] = {hue_size_, sat_size_};
	int ch[2] = {0, 1};
	
	cvtColor(img, hsv, CV_BGR2HSV);
  inRange(hsv, Scalar(0, 0, MIN(val_min_,val_max_)),
          Scalar(180, 256, MAX(val_min_, val_max_)), mask);
          
  Mat roi(hsv, detection), maskroi(mask, detection);
  calcHist(&roi, 1, ch, maskroi, hist_, 1, hist_size, pranges_);
  normalize(hist_, hist_, 0, 255, CV_MINMAX);
  
	calcBackProject(&hsv, 1, ch, hist_, backproj, pranges_);
	backproj &= mask;
	last_trackbox_ = CamShift(backproj, detection,
                            TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

	last_detection_ = last_trackbox_.boundingRect ();

	tracker_initialized = true;
  return last_trackbox_;
}

RotatedRect Tracker::camshift_track (Mat img) {
	Mat hsv, backproj, mask;
	int ch[2] = {0, 1};
	
	cvtColor(img, hsv, CV_BGR2HSV);
  inRange(hsv, Scalar(0, 0, MIN(val_min_,val_max_)),
          Scalar(180, 256, MAX(val_min_, val_max_)), mask);
  
	calcBackProject(&hsv, 1, ch, hist_, backproj, pranges_);
	backproj &= mask;
	
	Rect detection = last_trackbox_.boundingRect ();
	last_trackbox_ = CamShift(backproj, detection,
                            TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
  return last_trackbox_;
}

bool Tracker::initialized () {
	return tracker_initialized;
}
////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////


