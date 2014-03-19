// author : guido manfred
// mail : gmanfredi.mail@gmail.com

#ifndef RPNP_RPNP_H_
#define RPNP_RPNP_H_
// included just for the SVD function
#include <opencv2/core/core.hpp>

#include <iostream>
#include <fstream>
#include <cmath>

class rpnp {
 public:
  rpnp(void);
  ~rpnp();

  void set_internal_parameters(const double uc, const double vc,
			       const double fu, const double fv);
  void set_maximum_number_of_correspondences(const int n);
  void reset_correspondences(void);
  void add_correspondence(const double X, const double Y, const double Z,
                          const double u, const double v);
  double compute_pose(double R[3][3], double T[3]);
  void relative_error(const double Rtrue[3][3], const double ttrue[3],
                      const double Rest[3][3],  const double test[3],
                      double & rot_err, double & transl_err);
  void print_pose(const double R[3][3], const double t[3]);
  double reprojection_error(double R[3][3], double t[3]);

 private:
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
  double * XX_, *XXc_, * XXa_, * xx_, * xxv_;
  int maximum_number_of_correspondences_;
  int number_of_correspondences_;
  int i1_, i2_;
  double P0_[3];
  double R_[3][3];
  double D7_[7];
  double t2s_[7];
};

#endif
