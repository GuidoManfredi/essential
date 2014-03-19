// author : guido manfredi
// mail : gmanfredi.mail@gmail.com

#ifndef OPENCV_RPNP_HPP_
#define OPENCV_RPNP_HPP_
// included just for the SVD function
#include <opencv2/core/core.hpp>

#include <iostream>
#include <fstream>
#include <cmath>

class rpnp {
 public:
  rpnp(const cv::Mat& cameraMatrix, const cv::Mat& opoints, const cv::Mat& ipoints);
  ~rpnp();

  void compute_pose(cv::Mat& R, cv::Mat& t);
 private:
  template <typename T>
  void init_camera_parameters(const cv::Mat& cameraMatrix)
  {
    uc_ = cameraMatrix.at<T> (0, 2);
    vc_ = cameraMatrix.at<T> (1, 2);
    fu_ = cameraMatrix.at<T> (0, 0);
    fv_ = cameraMatrix.at<T> (1, 1);
  }
  template <typename OpointType, typename IpointType>
  void init_points(const cv::Mat& opoints, const cv::Mat& ipoints)
  {
      for(int i = 0; i < number_of_correspondences_; i++)
      {
          XX_[3 * i    ] = opoints.at<OpointType>(0,i).x;
          XX_[3 * i + 1] = opoints.at<OpointType>(0,i).y;
          XX_[3 * i + 2] = opoints.at<OpointType>(0,i).z;

          double u_norm = (ipoints.at<IpointType>(0,i).x - uc_)/fu_;
          double v_norm = (ipoints.at<IpointType>(0,i).y - vc_)/fv_;
          /*
          XX_[3 * i    ] = opoints.at<double>(i,0);
          XX_[3 * i + 1] = opoints.at<double>(i,1);
          XX_[3 * i + 2] = opoints.at<double>(i,2);

          double u_norm = (ipoints.at<double>(i,0) - uc_)/fu_;
          double v_norm = (ipoints.at<double>(i,1) - vc_)/fv_;
          */
          xx_[2 * i] = u_norm;
          xx_[2 * i + 1] = v_norm;
          xxv_[3 * i] = u_norm/norm(u_norm, v_norm, 1);
          xxv_[3 * i + 1] = v_norm/norm(u_norm, v_norm, 1);
          xxv_[3 * i + 2] = 1/norm(u_norm, v_norm, 1);
      }
  }

  double reprojection_error(const double R[3][3], const double t[3]);

  void select_longest_edge ();
  void compute_rotation_matrix ();
  void transform_points_to_new_frame ();
  void divide_into_3points_set ();
  void retrieve_local_minima ();
  double compute_camera_poses_minima (double R[3][3], double t[3]);

  void getp3p (double l1, double l2, double A5, double C1, double C2,
                double D1, double D2, double D3,
                double D4[5]);
  void getpoly7 (double F[5], double F7[8]);
  void calcampose (double * XXc, double * XXw,
                   double R[3][3], double t[3]);
  void polyval (double D6[7], double t2s[7],
                double F[7]);
  double norm (double u[3]);
  double norm (double u, double v, double w);
  void xcross(double a[3], double b[3],
              double c[3]);
  void roots (double D7[8],
              double real[7], double imag[7]);
  void eig (double DTD[6][6], double V[6][6], double D[6][6]);
  double max_array (double t2s[7]);
  void mat_to_quat(const double R[3][3], double q[4]);
  double dot(const double * v1, const double * v2);
  void log (std::string msg);

  double uc_, vc_, fu_, fv_;
  std::vector<double> XX_, XXc_, XXa_, xx_, xxv_;

  int number_of_correspondences_;
  int i1_, i2_;
  double P0_[3];
  double R_[3][3];
  double D7_[7];
  double t2s_[7];
};

#endif
