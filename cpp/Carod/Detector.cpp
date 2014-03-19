#include "Detector.h"

using namespace cv;
using namespace std;

void Detector::train_descs (vector<Mat> descs)
{
  _matcher.set_descriptors (descs);
}

void Detector::train_kpts_descs (vector< vector<KeyPoint> > kpts, vector<Mat> descs)
{
  _matcher.add_kpts_descriptors (kpts, descs);
}

Matx34d Detector::compute_pose (cv::Mat img, vector<Point3d> p3d)
{
  vector<cv::KeyPoint> kpts;
  Mat descs;
  vector<cv::DMatch> matches;
  _matcher.extract (img, kpts, descs);
  //cout << "Query descs size " << descs.size () << endl;
  _matcher.match (descs, matches);

  int n = matches.size ();
  //cout << "Matches : " << n << " for " << p3d.size () << " points." << endl;

  vector<int> kpts_idx(n);
  vector<int> p3d_idx(n);
  vector<KeyPoint> kpts_debug(n);
  for (size_t i=0; i<n; ++i) {
    kpts_idx[i] = matches[i].queryIdx;
    p3d_idx[i] = matches[i].trainIdx;
    //cout << p3d[ p3d_idx[i] ] << kpts[kpts_idx[i]].pt << endl;
  }
  /*
  string object_name = "cereales_nesquik";
  Mat img_train = imread ("/home/gmanfred/devel/these/projects/Carod/training_set/" + object_name + "/warp_2.jpg");
  _matcher.draw_matches (img, img_train, kpts, matches);
  */
  if (p3d_idx.size () > 4) {
    pose_estimator_->compute_pnp_ransac(p3d, kpts, p3d_idx, kpts_idx, 2.0, 200);
    //pose_estimator_->refine(p3d, kpts, p3d_idx, kpts_idx);
    P_ = pose_estimator_->get_P ();
//    cout << P_(0,0) << " " << P_(0,1) << " " << P_(0,2) << " " << P_(0,3) << endl;
//    cout << P_(1,0) << " " << P_(1,1) << " " << P_(1,2) << " " << P_(1,3) << endl;
//    cout << P_(2,0) << " " << P_(2,1) << " " << P_(2,2) << " " << P_(2,3) << endl;
  }
  else
    cout << "Not enough matches for pose estimation." << endl;

  return P_;
}

Detector::Detector (Mat K, Mat d) {
  pose_estimator_ = new PoseEstimator(K, d);
}

Detector::~Detector ()
{

}
