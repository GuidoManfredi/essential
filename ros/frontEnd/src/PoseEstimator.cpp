#include "PoseEstimator.h"

using namespace cv;
using namespace std;

PoseEstimator::PoseEstimator (Mat K): K_(K) {
  PNP_.set_internal_parameters(K_.at<double>(0, 2), K_.at<double>(1, 2),
                                K_.at<double>(0, 0), K_.at<double>(1, 1));
}

void PoseEstimator::compute_pnp_ransac (vector<Point3f> p3d, vector<Point2f> p2d,
                                        vector<int> p3d_idx,
                                        vector<int> p2d_idx,
                                        double max_reprojection_error, int max_its) {
  double R[3][3], best_R[3][3];
  double t[3], best_t[3];
  double reproj_err = 0.0;
  vector<int> inliers, best_inliers;
  int max_number_inliers = 0;
  int number_points = 4;

  PNP_.set_maximum_number_of_correspondences (number_points);
  for (int its=0; its < max_its; ++its) {
    add_random_correspondences (number_points, p3d, p2d, p3d_idx, p2d_idx);
    reproj_err = PNP_.compute_pose(R, t);
    inliers = get_inliers (p3d, p2d, p3d_idx, p2d_idx,
                           R, t,
                           max_reprojection_error);
    if (inliers.size() > max_number_inliers) {
      memcpy(best_R, R, sizeof(R));
      memcpy(best_t, t, sizeof(t));
      max_number_inliers = inliers.size ();
      best_inliers.swap (inliers);
      //cout << reproj_err << endl;
      //cout << max_number_inliers << endl;
      //max_its = update_its (number_inliers);
    }
  }
  cout << max_number_inliers << endl;
  vector<int> tmp_p3d_idx;
  vector<int> tmp_p2d_idx;
  tmp_p3d_idx.resize (max_number_inliers);
  tmp_p2d_idx.resize (max_number_inliers);
  for (size_t i=0; i<max_number_inliers; ++i) {
    tmp_p3d_idx[i] = p3d_idx[best_inliers[i]];
    tmp_p2d_idx[i] = p2d_idx[best_inliers[i]];
    //cout << i << " : " << p3d[p3d_idx[best_inliers[i]]] << endl;
  }

  PNP_.set_maximum_number_of_correspondences (max_number_inliers);
  add_correspondences (p3d, p2d, tmp_p3d_idx, tmp_p2d_idx);
  reproj_err = PNP_.compute_pose(best_R, best_t);
  //cout << reproj_err << endl;

  memcpy(R_, best_R, sizeof(best_R));
  memcpy(t_, best_t, sizeof(best_t));
}

void PoseEstimator::add_random_correspondences (int number_points,
                                                 vector<Point3f> p3d, vector<Point2f> p2d,
                                                 vector<int> p3d_idx, vector<int> p2d_idx) {
  PNP_.reset_correspondences();
  int rand_idx[number_points];
  for (int i=0; i<number_points;) {
    rand_idx[i] = Random::RandomInt(0, p3d_idx.size()-1);
    //cout << rand_idx[i] << endl;
    if ((i == 0) || (find(rand_idx, rand_idx + i, rand_idx[i]) == rand_idx + i)) {
      int idx3d = p3d_idx[rand_idx[i]];
      int idx2d = p2d_idx[rand_idx[i]];
      Point3f p3 = p3d[idx3d];
      Point2f p2 = p2d[idx2d];
      PNP_.add_correspondence (p3.x, p3.y, p3.z, p2.x, p2.y);
      ++i;
    }
  }
}

void PoseEstimator::add_correspondences (vector<Point3f> p3d, vector<Point2f> p2d,
                                          vector<int> p3d_idx, vector<int> p2d_idx) {
  PNP_.reset_correspondences();
  int n= p3d_idx.size();
  for (int i=0; i<n; ++i) {
      Point3f p3 = p3d[p3d_idx[i]];
      Point2f p2 = p2d[p2d_idx[i]];
      PNP_.add_correspondence (p3.x, p3.y, p3.z, p2.x, p2.y);
  }
}

vector<int> PoseEstimator::get_inliers(vector<Point3f> p3d, vector<Point2f> p2d,
                                        vector<int> p3d_idx, vector<int> p2d_idx,
                                        double R[3][3], double t[3],
                                        double max_reprojection_error) {
  vector<int> inliers;
  double p3[3];
  double err = 0;
  for(size_t i = 0; i < p3d_idx.size(); ++i) {
    p3[0]=p3d[p3d_idx[i]].x;  p3[1]=p3d[p3d_idx[i]].y;  p3[2]=p3d[p3d_idx[i]].z;
    double Xc = dot(R[0], p3) + t[0];
    double Yc = dot(R[1], p3) + t[1];
    double Zc = dot(R[2], p3) + t[2];
    double ue = K_.at<double>(0,2) + K_.at<double>(0,0) * Xc / Zc;
    double ve = K_.at<double>(1,2) + K_.at<double>(1,1) * Yc / Zc;
    double u = p2d[p2d_idx[i]].x;
    double v = p2d[p2d_idx[i]].y;
    err = sqrt((u-ue)*(u-ue)+(v-ve)*(v-ve));
    //cout << err << endl;
    if ( err <= max_reprojection_error)
      inliers.push_back (i);
  }

  return inliers;
}

vector<int> PoseEstimator::get_inliers(vector<Point3f> p3d, vector<Point2f> p2d,
                                        vector<int> p3d_idx, vector<int> p2d_idx,
                                        double max_reprojection_error) {
  vector<int> inliers;
  double p3[3];
  double err = 0;
  for(size_t i = 0; i < p3d_idx.size(); ++i) {
    p3[0]=p3d[p3d_idx[i]].x;  p3[1]=p3d[p3d_idx[i]].y;  p3[2]=p3d[p3d_idx[i]].z;
    double Xc = dot(R_[0], p3) + t_[0];
    double Yc = dot(R_[1], p3) + t_[1];
    double Zc = dot(R_[2], p3) + t_[2];
    double ue = K_.at<double>(0,2) + K_.at<double>(0,0) * Xc / Zc;
    double ve = K_.at<double>(1,2) + K_.at<double>(1,1) * Yc / Zc;
    double u = p2d[p2d_idx[i]].x;
    double v = p2d[p2d_idx[i]].y;
    err = sqrt((u-ue)*(u-ue)+(v-ve)*(v-ve));
    //cout << err << endl;
    if ( err <= max_reprojection_error)
      inliers.push_back (i);
  }

  return inliers;
}

void PoseEstimator::compute_pnp_standalone (vector<Point3f> p3d, vector<Point2f> p2d,
                                             vector<int> p3d_idx,
                                             vector<int> p2d_idx) {
  if (p3d_idx.size () < 4) {
    cout << "PoseEstimator: compute_pnp: not enough correspondances (only " << p3d_idx.size() << ")." << endl;
    return;
  }
  ofstream file;
  PNP_.set_maximum_number_of_correspondences(p3d_idx.size());
  PNP_.reset_correspondences();
  for (size_t i=0; i<p3d_idx.size(); ++i) {
    //cout << p3d[ p3d_idx[i] ].x << " " << p3d[ p3d_idx[i] ].y << " " << p3d[ p3d_idx[i] ].z << " "
    //     << p2d[ p2d_idx[i] ].x << " " << p2d[ p2d_idx[i] ].y << endl;
    PNP_.add_correspondence(p3d[ p3d_idx[i] ].x, p3d[ p3d_idx[i] ].y, p3d[ p3d_idx[i] ].z,
                            p2d[ p2d_idx[i] ].x, p2d[ p2d_idx[i] ].y);
  }
  file.close();

  double err = 0.0;
  err = PNP_.compute_pose(R_, t_);
  //cout << err << endl;
  //refine(p3d, p2d, p3d_idx, p2d_idx);
}

Matx34d PoseEstimator::getP ()
{
  Matx34d P;
  P(0,0) = R_[0][0]; P(0,1) = R_[0][1]; P(0,2) = R_[0][2]; P(0,3) = t_[0];
  P(1,0) = R_[1][0]; P(1,1) = R_[1][1]; P(1,2) = R_[1][2]; P(1,3) = t_[1];
  P(2,0) = R_[2][0]; P(2,1) = R_[2][1]; P(2,2) = R_[2][2]; P(2,3) = t_[2];

  return P;
}
