#include "PoseEstimator.h"

using namespace cv;
using namespace std;

PoseEstimator::PoseEstimator (Mat K): K_(K) {
  PNP_.set_internal_parameters(K_.at<double>(0, 2), K_.at<double>(1, 2),
                                K_.at<double>(0, 0), K_.at<double>(1, 1));
}

void PoseEstimator::compute_pnp_ransac (vector<Point3d> p3d, vector<KeyPoint> kpts,
                                        vector<int> p3d_idx,
                                        vector<int> kpts_idx,
                                        double max_reprojection_error, int max_its) {
  double R[3][3], best_R[3][3];
  double t[3], best_t[3];
  double reproj_err = 0.0;
  vector<int> inliers, best_inliers;
  int max_number_inliers = 0;
  int number_points = 4;

  PNP_.set_maximum_number_of_correspondences (number_points);
  for (int its=0; its < max_its; ++its) {
    add_random_correspondences (number_points, p3d, kpts, p3d_idx, kpts_idx);
    reproj_err = PNP_.compute_pose(R, t);
    inliers = get_inliers (p3d, kpts, p3d_idx, kpts_idx,
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
  vector<int> tmp_kpts_idx;
  tmp_p3d_idx.resize (max_number_inliers);
  tmp_kpts_idx.resize (max_number_inliers);
  for (size_t i=0; i<max_number_inliers; ++i) {
    tmp_p3d_idx[i] = p3d_idx[best_inliers[i]];
    tmp_kpts_idx[i] = kpts_idx[best_inliers[i]];
    //cout << i << " : " << p3d[p3d_idx[best_inliers[i]]] << endl;
  }

  PNP_.set_maximum_number_of_correspondences (max_number_inliers);
  add_correspondences (p3d, kpts, tmp_p3d_idx, tmp_kpts_idx);
  reproj_err = PNP_.compute_pose(best_R, best_t);
  //cout << reproj_err << endl;

  memcpy(R_, best_R, sizeof(best_R));
  memcpy(t_, best_t, sizeof(best_t));
}

void PoseEstimator::add_random_correspondences (int number_points,
                                                 vector<Point3d> p3d, vector<KeyPoint> kpts,
                                                 vector<int> p3d_idx, vector<int> kpts_idx) {
  PNP_.reset_correspondences();
  int rand_idx[number_points];
  for (int i=0; i<number_points;) {
    rand_idx[i] = Random::RandomInt(0, p3d_idx.size()-1);
    //cout << rand_idx[i] << endl;
    if ((i == 0) || (find(rand_idx, rand_idx + i, rand_idx[i]) == rand_idx + i)) {
      int idx3d = p3d_idx[rand_idx[i]];
      int idx2d = kpts_idx[rand_idx[i]];
      Point3d p3 = p3d[idx3d];
      Point2d p2 = kpts[idx2d].pt;
      PNP_.add_correspondence (p3.x, p3.y, p3.z, p2.x, p2.y);
      ++i;
    }
  }
}

void PoseEstimator::add_correspondences (vector<Point3d> p3d, vector<KeyPoint> kpts,
                                          vector<int> p3d_idx, vector<int> kpts_idx) {
  PNP_.reset_correspondences();
  int n= p3d_idx.size();
  int rand_idx[n];
  for (int i=0; i<n; ++i) {
      Point3d p3 = p3d[p3d_idx[i]];
      Point2d p2 = kpts[kpts_idx[i]].pt;
      PNP_.add_correspondence (p3.x, p3.y, p3.z, p2.x, p2.y);
  }
}

vector<int> PoseEstimator::get_inliers(vector<Point3d> p3d, vector<KeyPoint> kpts,
                                        vector<int> p3d_idx, vector<int> kpts_idx,
                                        double R[3][3], double t[3],
                                        double max_reprojection_error) {
  vector<int> inliers;
  double p3[3];
  int i = 0;
  double err = 0;
  for(int i = 0; i < p3d_idx.size(); ++i) {
    p3[0]=p3d[p3d_idx[i]].x;  p3[1]=p3d[p3d_idx[i]].y;  p3[2]=p3d[p3d_idx[i]].z;
    double Xc = dot(R[0], p3) + t[0];
    double Yc = dot(R[1], p3) + t[1];
    double Zc = dot(R[2], p3) + t[2];
    double ue = K_.at<double>(0,2) + K_.at<double>(0,0) * Xc / Zc;
    double ve = K_.at<double>(1,2) + K_.at<double>(1,1) * Yc / Zc;
    double u = kpts[kpts_idx[i]].pt.x;
    double v = kpts[kpts_idx[i]].pt.y;
    err = sqrt((u-ue)*(u-ue)+(v-ve)*(v-ve));
    //cout << err << endl;
    if ( err <= max_reprojection_error)
      inliers.push_back (i);
  }

  return inliers;
}

int PoseEstimator::get_number_outliers(vector<Point3d> p3d, vector<KeyPoint> kpts,
                                        vector<int> p3d_idx, vector<int> kpts_idx,
                                        double R[3][3], double t[3],
                                        double max_reprojection_error) {
  int num_outliers = 0;
  double p3[3];
  int i = 0;
  double err = 0;
  //PNP_.print_pose(R, t);
  //cout << K_ << endl;
  for(int i = 0; i < p3d_idx.size(); ++i) {
    p3[0]=p3d[p3d_idx[i]].x;  p3[1]=p3d[p3d_idx[i]].y;  p3[2]=p3d[p3d_idx[i]].z;
    //cout << p3[0] << " " << p3[1] << " " << p3[2] << " " << kpts[kpts_idx[i]].pt.x << " " << kpts[kpts_idx[i]].pt.y << endl;
    double Xc = dot(R[0], p3) + t[0];
    double Yc = dot(R[1], p3) + t[1];
    double Zc = dot(R[2], p3) + t[2];
    //cout << Xc << " " << Yc << " " << Zc << " " << K_.at<double>(0,0) << " " << K_.at<double>(1,1) << endl;
    double ue = K_.at<double>(0,2) + K_.at<double>(0,0) * Xc / Zc;
    double ve = K_.at<double>(1,2) + K_.at<double>(1,1) * Yc / Zc;
    double u = kpts[kpts_idx[i]].pt.x;
    double v = kpts[kpts_idx[i]].pt.y;
    /*
    if (Zc < 0)
      err = sqrt(2*(-Zc + 10)*(-Zc +10));
    else
    */
    err = sqrt((u-ue)*(u-ue)+(v-ve)*(v-ve));
    //cout << err << endl;
    if ( err > max_reprojection_error)
      ++num_outliers;
  }
  //cout << num_outliers << endl;
  return num_outliers;
}

void PoseEstimator::compute_pnp_standalone (vector<Point3d> p3d, vector<KeyPoint> kpts,
                                             vector<int> p3d_idx,
                                             vector<int> kpts_idx) {
  if (p3d_idx.size () < 4) {
    cout << "PoseEstimator: compute_pnp: not enough correspondances (only " << p3d_idx.size() << ")." << endl;
    return;
  }
  ofstream file;
  PNP_.set_maximum_number_of_correspondences(p3d_idx.size());
  PNP_.reset_correspondences();
  for (size_t i=0; i<p3d_idx.size(); ++i) {
    //cout << p3d[ p3d_idx[i] ].x << " " << p3d[ p3d_idx[i] ].y << " " << p3d[ p3d_idx[i] ].z << " "
    //     << kpts[ kpts_idx[i] ].pt.x << " " << kpts[ kpts_idx[i] ].pt.y << endl;
    PNP_.add_correspondence(p3d[ p3d_idx[i] ].x, p3d[ p3d_idx[i] ].y, p3d[ p3d_idx[i] ].z,
                            kpts[ kpts_idx[i] ].pt.x, kpts[ kpts_idx[i] ].pt.y);
  }
  file.close();

  double err = 0.0;
  err = PNP_.compute_pose(R_, t_);
  //cout << err << endl;
  //refine(p3d, kpts, p3d_idx, kpts_idx);
}

void PoseEstimator::refine (vector<Point3d> p3d, vector<KeyPoint> kpts,
                            vector<int> p3d_idx,
                            vector<int> kpts_idx) {
  /*
  OptimizationData dat;
  dat.cal[0]=K_.at<float>(0, 2);
  dat.cal[1]=K_.at<float>(1, 2);
  dat.cal[2]=K_.at<float>(0, 0);
  dat.cal[3]=K_.at<float>(1, 1);
  dat.cal[4]=0.0;//d_.at<float>(0,0);
  dat.cal[5]=0.0;//d_.at<float>(0,1);
  dat.cam(0,0)=R_[0][0]; dat.cam(0,1)=R_[0][1]; dat.cam(0,2)=R_[0][2];
  dat.cam(1,0)=R_[1][0]; dat.cam(1,1)=R_[1][1]; dat.cam(1,2)=R_[1][2];
  dat.cam(2,0)=R_[2][0]; dat.cam(2,1)=R_[2][1]; dat.cam(2,2)=R_[2][2];

  dat.cam(3,0)=0; dat.cam(3,1)=0; dat.cam(3,2)=0; dat.cam(3,3)=1;
  dat.cam(0,3)=t_[0];
  dat.cam(1,3)=t_[1];
  dat.cam(2,3)=t_[2];

  Vector2 pt2;
  Vector3 pt3;
  for(unsigned int i = 0; i < p3d_idx.size(); ++i) {
    Point3d tmp_p3d = p3d[ p3d_idx[i] ];
    pt3[0]=tmp_p3d.x;
    pt3[1]=tmp_p3d.y;
    pt3[2]=tmp_p3d.z;
    dat.p3d.push_back(pt3);
    KeyPoint tmp_p2d = kpts[ kpts_idx[i] ];
    pt2[0]=tmp_p2d.pt.x;
    pt2[1]=tmp_p2d.pt.y;
    dat.p2d.push_back(pt2);
    //cout << pt3[0] << " "  << pt3[1] << " " << pt3[2] << " " << pt2[0] << " " << pt2[1] << endl;
  }
  //cerr<<"Tcm=["<<dat.cam<<"];\n";
  poseOptimization(dat);

  R_[0][0]=dat.cam(0,0); R_[0][1]=dat.cam(0,1); R_[0][2]=dat.cam(0,2);
  R_[1][0]=dat.cam(1,0); R_[1][1]=dat.cam(1,1); R_[1][2]=dat.cam(1,2);
  R_[2][0]=dat.cam(2,0); R_[2][1]=dat.cam(2,1); R_[2][2]=dat.cam(2,2);
  t_[0]=dat.cam(0,3); t_[1]=dat.cam(1,3); t_[2]=dat.cam(2,3);
  */
}

Mat PoseEstimator::get_P ()
{
  Matx34d P;
  P(0,0) = R_[0][0]; P(0,1) = R_[0][1]; P(0,2) = R_[0][2]; P(0,3) = t_[0];
  P(1,0) = R_[1][0]; P(1,1) = R_[1][1]; P(1,2) = R_[1][2]; P(1,3) = t_[1];
  P(2,0) = R_[2][0]; P(2,1) = R_[2][1]; P(2,2) = R_[2][2]; P(2,3) = t_[2];

  return Mat(P);
}
