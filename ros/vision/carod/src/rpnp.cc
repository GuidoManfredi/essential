// author : guido manfred
// mail : gmanfredi.mail@gmail.com

#include "rpnp.h"

using namespace std;
using namespace cv;

rpnp::rpnp () {
  maximum_number_of_correspondences_ = 0;
  number_of_correspondences_ = 0;

  XX_ = 0;
  XXa_ = 0;
  XXc_ = 0;
  xx_ = 0;
  xxv_ = 0;

  i1_ = 0;
  i2_ = 0;

  for (int i=0; i< 8; ++i) {
    D7_[i] = 0;
    t2s_[i] = 0;
  }
  for(int i=0; i<3; ++i) {
    P0_[i] = 0;
    for(int j=0; j<3; ++j) {
      R_[i][j] = 0;
    }
  }
}

rpnp::~rpnp () {
}

void rpnp::set_internal_parameters(double uc, double vc,
                                   double fu, double fv) {
  uc_ = uc;
  vc_ = vc;
  fu_ = fu;
  fv_ = fv;
  /*
  cout << fu_ << " " << 0 << " " << uc_ << endl
       << 0 << " " << fv_ << " " << vc_ << endl
       << 0 << " " << 0 << " " << 1 << endl;
  */
}

void rpnp::set_maximum_number_of_correspondences(int n) {
  if (maximum_number_of_correspondences_ < n) {
    if (XX_ != 0) delete [] XX_;
    if (XXa_ != 0) delete [] XXa_;
    if (XXc_ != 0) delete [] XXc_;
    if (xx_ != 0) delete [] xx_;
    if (xxv_ != 0) delete [] xxv_;

    maximum_number_of_correspondences_ = n;
    XX_ = new double[3 * maximum_number_of_correspondences_];
    XXa_ = new double[3 * maximum_number_of_correspondences_];
    XXc_ = new double[3 * maximum_number_of_correspondences_];
    xx_ = new double[2 * maximum_number_of_correspondences_];
    xxv_ = new double[3 * maximum_number_of_correspondences_];
  }
}

void rpnp::reset_correspondences(void) {
  number_of_correspondences_ = 0;
  for (int i=0; i< 8; ++i) {
    D7_[i] = 0;
    t2s_[i] = 0;
  }
  for(int i=0; i<3; ++i) {
    P0_[i] = 0;
    for(int j=0; j<3; ++j) {
      R_[i][j] = 0;
    }
  }
}

void rpnp::add_correspondence(double X, double Y, double Z,
                               double u, double v) {
  int n = number_of_correspondences_;
  XX_[3 * n    ] = X;
  XX_[3 * n + 1] = Y;
  XX_[3 * n + 2] = Z;
  /*
  cout << XX_[3 * number_of_correspondences_    ] << " "
       << XX_[3 * number_of_correspondences_ + 1] << " "
       << XX_[3 * number_of_correspondences_ + 2] << endl;
  */

  double u_norm = (u - uc_)/fu_;
  double v_norm = (v - vc_)/fv_;
  xx_[2 * n] = u_norm;
  xx_[2 * n + 1] = v_norm;
  /*
  cout << xx_[2 * number_of_correspondences_    ] << " "
       << xx_[2 * number_of_correspondences_ + 1] << endl;
  */
  xxv_[3 * n] = u_norm/norm(u_norm, v_norm, 1);
  xxv_[3 * n + 1] = v_norm/norm(u_norm, v_norm, 1);
  xxv_[3 * n + 2] = 1/norm(u_norm, v_norm, 1);
  /*
  xx_[2 * n] = u;
  xx_[2 * n + 1] = v;

  xxv_[3 * number_of_correspondences_    ] = u/norm(u, v, 1);
  xxv_[3 * number_of_correspondences_ + 1] = v/norm(u, v, 1);
  xxv_[3 * number_of_correspondences_ + 2] = 1/norm(u, v, 1);
  */
  /*
  cout << xxv_[3 * number_of_correspondences_    ] << " "
       << xxv_[3 * number_of_correspondences_ + 1] << " "
       << xxv_[3 * number_of_correspondences_ + 2] << endl;
  */
  ++number_of_correspondences_;
}

double rpnp::compute_pose(double R[3][3], double t[3]) {
  select_longest_edge ();
  compute_rotation_matrix ();
  transform_points_to_new_frame ();
  divide_into_3points_set ();
  retrieve_local_minima ();
  return compute_camera_poses_minima (R, t);
  //cout << t[0] << " " << t[1] << " " << t[2] << endl;
}

void rpnp::print_pose(const double R[3][3], const double t[3])
{
  cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
  cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
  cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}

void rpnp::relative_error(const double Rtrue[3][3], const double ttrue[3],
                          const double Rest[3][3],  const double test[3],
                          double & rot_err, double & transl_err)
{
  double qtrue[4], qest[4];

  mat_to_quat(Rtrue, qtrue);
  mat_to_quat(Rest, qest);

  double rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
			 (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
			 (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
			 (qtrue[3] - qest[3]) * (qtrue[3] - qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  double rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
			 (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
			 (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
			 (qtrue[3] + qest[3]) * (qtrue[3] + qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  rot_err = min(rot_err1, rot_err2);

  transl_err =
    sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
	 (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
	 (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
    sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}

double rpnp::reprojection_error(double R[3][3], double t[3])
{
  double sum2 = 0.0;

  for(int i = 0; i < number_of_correspondences_; i++) {
    double * pw = XX_ + 3 * i;
    double Xc = dot(R[0], pw) + t[0];
    double Yc = dot(R[1], pw) + t[1];
    double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
    double ue = uc_ + fu_ * Xc * inv_Zc;
    double ve = vc_ + fv_ * Yc * inv_Zc;
    double u = uc_ + xx_[2 * i] * fu_, v = vc_ + xx_[2 * i + 1] * fv_;

    sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
  }
  //cout << sum2 / number_of_correspondences_ << endl;
  return sum2 / number_of_correspondences_;
}

////////////////////////////////////////////////////////////////////////////////
//                                PRIVATE METHODS
////////////////////////////////////////////////////////////////////////////////
void rpnp::select_longest_edge () {
  double length_min = 1000, length = 0;
  for(int i=0; i<number_of_correspondences_-1; ++i) {
    for(int j=i+1; j<number_of_correspondences_; ++j) {
      length = xxv_[3*i]*xxv_[3*j]+xxv_[3*i+1]*xxv_[3*j+1]+xxv_[3*i+2]*xxv_[3*j+2];
      //cout << length << endl;
      if (length < length_min) {
        i1_ = i;
        i2_ = j;
        length_min = length;
      }
    }
  }
  //cout << length_min << endl;
}

void rpnp::compute_rotation_matrix () {
  double normx, normy, normz;
  double P1[3], P2[3], x[3];
  P0_[0] = (XX_[3*i1_]+XX_[3*i2_])/2;
  P0_[1] = (XX_[3*i1_+1]+XX_[3*i2_+1])/2;
  P0_[2] = (XX_[3*i1_+2]+XX_[3*i2_+2])/2;
  x[0] = XX_[3*i2_] - P0_[0];
  x[1] = XX_[3*i2_+1] - P0_[1];
  x[2] = XX_[3*i2_+2] - P0_[2];
  normx = norm(x);
  x[0] /= normx;
  x[1] /= normx;
  x[2] /= normx;

  double y[3], z[3];
  double u2[3] = {0, 1, 0};
  double u3[3] = {0, 0, 1};
  if(fabs(x[0]) < fabs(x[2])) {
    xcross (x, u2, z);
    normz = norm(z);
    z[0] /= normz;    z[1] /= normz;    z[2] /= normz;
    xcross (z, x, y);
    normy = norm(y);
    y[0] /= normy;    y[1] /= normy;    y[2] /= normy;
  }
  else {
    xcross (u3, x, y);
    normy = norm(y);
    y[0] /= normy;    y[1] /= normy;    y[2] /= normy;
    xcross (x, y, z);
    normz = norm(z);
    z[0] /= normz;    z[1] /= normz;    z[2] /= normz;
  }

  R_[0][0] = x[0];  R_[0][1] = y[0];  R_[0][2] = z[0];
  R_[1][0] = x[1];  R_[1][1] = y[1];  R_[1][2] = z[1];
  R_[2][0] = x[2];  R_[2][1] = y[2];  R_[2][2] = z[2];

  /*
  cout << R_[0][0] << " " << R_[0][1] << " " << R_[0][2] << endl
       << R_[1][0] << " " << R_[1][1] << " " << R_[1][2] << endl
       << R_[2][0] << " " << R_[2][1] << " " << R_[2][2] << endl;
  */
}

void rpnp::transform_points_to_new_frame () {
  for (int i=0; i<number_of_correspondences_; ++i) {
    XXa_[3*i] = (XX_[3*i] - P0_[0])*R_[0][0] + (XX_[3*i+1] - P0_[1])*R_[1][0] + (XX_[3*i+2] - P0_[2])*R_[2][0];
    XXa_[3*i+1] = (XX_[3*i] - P0_[0])*R_[0][1] + (XX_[3*i+1] - P0_[1])*R_[1][1] + (XX_[3*i+2] - P0_[2])*R_[2][1];
    XXa_[3*i+2] = (XX_[3*i] - P0_[0])*R_[0][2] + (XX_[3*i+1] - P0_[1])*R_[1][2] + (XX_[3*i+2] - P0_[2])*R_[2][2];
    //cout << XXa_[3*i] << " " << XXa_[3*i+1] << " " << XXa_[3*i+2] << endl;
  }
}

// and determining deviation of cost function
void rpnp::divide_into_3points_set () {
  double cg1, cg2, cg3, sg1, sg2;
  double D1, D2, D3;
  double D4[number_of_correspondences_-2][5];
  cg1 = xxv_[3*i1_]*xxv_[3*i2_]
      + xxv_[3*i1_+1]*xxv_[3*i2_+1]
      + xxv_[3*i1_+2]*xxv_[3*i2_+2];
  sg1 = sqrt (1 - cg1*cg1);
  D1 = norm(XX_[3*i1_] - XX_[3*i2_],
            XX_[3*i1_+1] - XX_[3*i2_+1],
            XX_[3*i1_+2] - XX_[3*i2_+2]);
  for (int i=0, j=0; i<number_of_correspondences_; ++i) {
    if (i == i1_ || i == i2_)
      continue;
    cg2 = xxv_[3*i1_]*xxv_[3*i]
        + xxv_[3*i1_+1]*xxv_[3*i+1]
        + xxv_[3*i1_+2]*xxv_[3*i+2];
    cg3 = xxv_[3*i2_]*xxv_[3*i]
        + xxv_[3*i2_+1]*xxv_[3*i+1]
        + xxv_[3*i2_+2]*xxv_[3*i+2];
    sg2 = sqrt (1-cg2*cg2);
    D2 = norm (XX_[3*i1_] - XX_[3*i],
               XX_[3*i1_+1] - XX_[3*i+1],
               XX_[3*i1_+2] - XX_[3*i+2]);
    D3 = norm (XX_[3*i] - XX_[3*i2_],
               XX_[3*i+1] - XX_[3*i2_+1],
               XX_[3*i+2] - XX_[3*i2_+2]);
    getp3p(cg1, cg2, cg3, sg1, sg2, D1, D2, D3, D4[j]);
    //cout << D4[j][0] << " " << D4[j][1] << " " << D4[j][2] << " " << D4[j][3] << " " << D4[j][4] << " " << endl;
    ++j;
  }

  double D[8];
  for (int i=0; i<number_of_correspondences_-2; ++i) {
    getpoly7(D4[i], D);
    D7_[0] += D[0];
    D7_[1] += D[1];
    D7_[2] += D[2];
    D7_[3] += D[3];
    D7_[4] += D[4];
    D7_[5] += D[5];
    D7_[6] += D[6];
    D7_[7] += D[7];
  }
  //cout << D7_[0] << " " << D7_[1] << " " << D7_[2] << " " << D7_[3] << " " << D7_[4] << " " << D7_[5] << " " << D7_[6] << " " << D7_[7] << endl;
}

void rpnp::retrieve_local_minima () {
  double real[7], imag[7];
  roots (D7_, real, imag);
  double max_real_root = max_array(real);
  //cout << max_real_root << endl;

  for (int i=0; i<7; ++i) {
    //cout << real[i] << " " << imag[i] << endl;
    if (fabs(imag[i])/max_real_root < 0.001)
      t2s_[i] = real[i];
    else
      t2s_[i] = 0.0;
  }

  double D6[7];
  D6[0] = 7*D7_[0];
  D6[1] = 6*D7_[1];
  D6[2] = 5*D7_[2];
  D6[3] = 4*D7_[3];
  D6[4] = 3*D7_[4];
  D6[5] = 2*D7_[5];
  D6[6] = D7_[6];
  //cout << D6[0] << " " << D6[1] << " " << D6[2] << " " << D6[3] << " " << D6[4] << " " << D6[5] << " " << D6[6] << endl;
  double F6[7];
  polyval(D6, t2s_, F6);
  // TODO verify this correspond to matlab. Can't read the matlab code. line 130
  for (int i=0; i<7; ++i) {
    if (F6[i] > 0)
      t2s_[i] = real[i];
    else
      t2s_[i] = 0.0;
  }
  //cout << t2s_[0] << " " << t2s_[1] << " " << t2s_[2] << " " << t2s_[3] << " " << t2s_[4] << " " << t2s_[5] << " " << t2s_[6] << endl;
  if(t2s_[0] == 0 && t2s_[1] == 0 && t2s_[2] == 0 && t2s_[3] == 0 && t2s_[4] == 0
     && t2s_[5] == 0 && t2s_[6] == 0) {
    cout << "No solution" << endl;
    return;
  }
}

double rpnp::compute_camera_poses_minima (double best_R[3][3], double best_t[3]) {
  double min_error = 1000;

  double cg1 = xxv_[3*i1_]*xxv_[3*i2_]
              + xxv_[3*i1_+1]*xxv_[3*i2_+1]
              + xxv_[3*i1_+2]*xxv_[3*i2_+2];
  //cout << cg1 << endl;
  double x[3];
  for (int i=0; i<7; ++i) {
    //cout << t2s_[i] << endl;
    if (t2s_[i] != 0) {
      double R[3][3];
      double t[3];
      double d2 = cg1 + t2s_[i];
      x[0] = d2 * xxv_[3*i2_] - xxv_[3*i1_];
      x[1] = d2 * xxv_[3*i2_+1] - xxv_[3*i1_+1];
      x[2] = d2 * xxv_[3*i2_+2] - xxv_[3*i1_+2];

      double normx = norm(x);
      x[0] /= normx;  x[1] /= normx;  x[2] /= normx;

      double y[3], z[3];
      double u2[3] = {0, 1, 0};
      double u3[3] = {0, 0, 1};
      double normy, normz;
      if (fabs(x[1]) < fabs(x[2])) {
        xcross (x, u2, z);
        normz = norm(z);
        z[0] /= normz;    z[1] /= normz;    z[2] /= normz;
        xcross (z, x, y);
        normy = norm(y);
        y[0] /= normy;    y[1] /= normy;    y[2] /= normy;
      }
      else {
        xcross (u3, x, y);
        normy = norm(y);
        y[0] /= normy;    y[1] /= normy;    y[2] /= normy;
        xcross (x, y, z);
        normz = norm(z);
        z[0] /= normz;    z[1] /= normz;    z[2] /= normz;
      }

      double r[3][3];
      r[0][0] = x[0];  r[1][0] = y[0];  r[2][0] = z[0];
      r[0][1] = x[1];  r[1][1] = y[1];  r[2][1] = z[1];
      r[0][2] = x[2];  r[1][2] = y[2];  r[2][2] = z[2];
      /*
      cout << r[0][0] << " " << r[0][1] << " " << r[0][2] << endl
            << r[1][0] << " " << r[1][1] << " " << r[1][2] << endl
            << r[2][0] << " " << r[2][1] << " " << r[2][2] << endl;
      */

      double D[2*number_of_correspondences_][6];
      for(int j=0; j<number_of_correspondences_; ++j) {
        D[2*j][0] = -r[1][0]*XXa_[3*j+1]+xx_[2*j]*(r[1][2]*XXa_[3*j+1]+r[2][2]*XXa_[3*j+2])-r[2][0]*XXa_[3*j+2];
        D[2*j][1] = -r[2][0]*XXa_[3*j+1]+xx_[2*j]*(r[2][2]*XXa_[3*j+1]-r[1][2]*XXa_[3*j+2])+r[1][0]*XXa_[3*j+2];
        D[2*j][2] = -1;
        D[2*j][3] = 0;
        D[2*j][4] = xx_[2*j];
        D[2*j][5] = xx_[2*j]*r[0][2]*XXa_[3*j]-r[0][0]*XXa_[3*j];
        D[2*j+1][0] = -r[1][1]*XXa_[3*j+1]+xx_[2*j+1]*(r[1][2]*XXa_[3*j+1]+r[2][2]*XXa_[3*j+2])-r[2][1]*XXa_[3*j+2];
        D[2*j+1][1] = -r[2][1]*XXa_[3*j+1]+xx_[2*j+1]*(r[2][2]*XXa_[3*j+1]-r[1][2]*XXa_[3*j+2])+r[1][1]*XXa_[3*j+2];
        D[2*j+1][2] = 0;
        D[2*j+1][3] = -1;
        D[2*j+1][4] = xx_[2*j+1];
        D[2*j+1][5] = xx_[2*j+1]*r[0][2]*XXa_[3*j]-r[0][1]*XXa_[3*j];
        /*
        cout << D[2*j][0] << " " << D[2*j][1] << " " << D[2*j][2] << " " << D[2*j][3] << " " << D[2*j][4] << " " << D[2*j][5] << endl;
        cout << D[2*j+1][0] << " " << D[2*j+1][1] << " " << D[2*j+1][2] << " " << D[2*j+1][3] << " " << D[2*j+1][4] << " " << D[2*j+1][5] << endl;
        */
      }

      double DTD[6][6] = {0};
      for (int i=0; i<6;++i) {
        for (int j=0; j<6;++j) {
          for (int k=0; k<2*number_of_correspondences_; ++k) {
            DTD[i][j] += D[k][i]*D[k][j];
          }
        }
      }

      double V[6][6], Diag[6][6];
      eig (DTD, V, Diag);
      /*
      for (int i=0; i<6; ++i)
        cout << DTD[i][0] << " " << DTD[i][1] << " " << DTD[i][2] << " " << DTD[i][3] << " " << DTD[i][4] << " " << DTD[i][5] << " " << endl;
      cout << endl;
      */
      double c, s;
      c = V[0][0]/V[5][0];
      s = V[1][0]/V[5][0];
      t[0] = V[2][0]/V[5][0];
      t[1] = V[3][0]/V[5][0];
      t[2] = V[4][0]/V[5][0];
      //cout << c << " " << s << " " << t[0] << " " << t[1] << " " << t[2] << endl;
      double XXcs[number_of_correspondences_][3];
      for (int i=0; i<number_of_correspondences_; ++i) {
        XXcs[i][0] = r[0][0]*XXa_[3*i]
                   + (r[1][0]*c+r[2][0]*s)*XXa_[3*i+1]
                   + (-r[1][0]*s+r[2][0]*c)*XXa_[3*i+2]
                   + t[0];
        XXcs[i][1] = r[0][1]*XXa_[3*i]
                   + (r[1][1]*c+r[2][1]*s)*XXa_[3*i+1]
                   + (-r[1][1]*s+r[2][1]*c)*XXa_[3*i+2]
                   + t[1];
        XXcs[i][2] = r[0][2]*XXa_[3*i]
                   + (r[1][2]*c+r[2][2]*s)*XXa_[3*i+1]
                   + (-r[1][2]*s+r[2][2]*c)*XXa_[3*i+2]
                   + t[2];
        //cout << XXcs[i][0] << " " << XXcs[i][1] << " " << XXcs[i][2] << endl;
      }

      for(int i=0; i<number_of_correspondences_; ++i) {
        XXc_[3*i] = xxv_[3*i] * norm(XXcs[i]);
        XXc_[3*i+1] = xxv_[3*i+1] * norm(XXcs[i]);
        XXc_[3*i+2] = xxv_[3*i+2] * norm(XXcs[i]);
        //cout << XXc_[3*i] << " " << XXc_[3*i+1] << " " << XXc_[3*i+2] << endl;
      }
      calcampose ((double*)XXc_, XX_, R, t);
      double error = reprojection_error (R, t);
      if (error < min_error) {
        for (int i=0; i<3; ++i) {
          for(int j=0; j<3; ++j)
            best_R[i][j] = R[i][j];
          best_t[i] = t[i];
        }
        min_error = error;
      }
    }
  }
  if (min_error > 5.0) {
    //cout << "Warning: high reprojection error: " << min_error;
  }
  return min_error;
}

void rpnp::getp3p (double l1, double l2, double A5, double C1, double C2,
                    double D1, double D2, double D3,
                    double D4[5]) {
  double A1 = (D2/D1)*(D2/D1);
  double A2 = A1*C1*C1 - C2*C2;
  double A3 = l2*A5 - l1;
  double A4 = l1*A5 - l2;
  double A6 = (D3*D3-D1*D1-D2*D2)/(2*D1*D1);
  double A7 = 1 - l1*l1 - l2*l2 + l1*l2*A5 + A6*C1*C1;

  //cout << A1 << " " << A2 << " " << A3 << " " << A4 << " " << A5 << " " << A6 << " " << A7 << endl;
  D4[0] = A6*A6 - A1*A5*A5;
  D4[1] = 2*(A3*A6-A1*A4*A5);
  D4[2] = A3*A3+2*A6*A7-A1*A4*A4-A2*A5*A5;
  D4[3] = 2*(A3*A7-A2*A4*A5);
  D4[4] = A7*A7-A2*A4*A4;
}

void rpnp::getpoly7 (double F[5], double F7[8]) {
  //cout << F[0] << " " << F[1] << " " << F[2] << " " << F[3] << " " << F[4] << endl;
  F7[0] = 4*F[0]*F[0];
  F7[1] = 7*F[1]*F[0];
  F7[2] = 6*F[2]*F[0] + 3*F[1]*F[1];
  F7[3] = 5*F[3]*F[0] + 5*F[2]*F[1];
  F7[4] = 4*F[4]*F[0] + 4*F[3]*F[1] + 2*F[2]*F[2];
  F7[5] = 3*F[4]*F[1] + 3*F[3]*F[2];
  F7[6] = 2*F[4]*F[2] + F[3]*F[3];
  F7[7] = F[4]*F[3];
  /*
  cout << F7[0] << " " << F7[1] << " " << F7[2] << " " << F7[3] << " " << F7[4]
       << F7[5] << " " << F7[6] << " " << F7[7] << " " << endl;
  */
}

void rpnp::calcampose (double * XXc, double * XXw,
                        double R[3][3], double t[3]) {
  int n = number_of_correspondences_;
  Mat X = Mat(n, 3, CV_64F, XXw);
  Mat Y = Mat(n, 3, CV_64F, XXc);
  Mat K = Mat::eye(n, n, CV_64F)
        - Mat::ones(n, n, CV_64F)/n;
  double meanx[3], meany[3], sigmx2 = 0;
  for(int i=0; i<n; ++i) {
    meanx[0] += X.at<double>(i,0);    meanx[1] += X.at<double>(i,1);    meanx[2] += X.at<double>(i,2);
    meany[0] += Y.at<double>(i,0);    meany[1] += Y.at<double>(i,1);    meany[2] += Y.at<double>(i,2);
  }
  meanx[0] /= n;  meanx[1] /= n;  meanx[2] /= n;
  meany[0] /= n;  meany[1] /= n;  meany[2] /= n;
  //cout << meanx[0] << " " << meanx[1] << " " << meanx[2] << endl;
  Mat XK= (X.t()*K).mul(X.t()*K);
  for (int k=0; k<n; ++k)
    for (int i=0; i<3; ++i)
      sigmx2 += XK.at<double>(i, k);
  sigmx2 /= n;
  //cout << sigmx2 << endl;
  Mat SXY = Y.t()*K*X/n;
  Mat U, D, Vt;
  //cout << X.t() << endl << Y.t() << endl;
  SVD::compute(SXY, D, U, Vt);
  //cout << U << endl << Y << endl << D << endl << Vt << endl;
  Mat S = Mat::eye(3, 3, CV_64F);
  if (determinant(SXY) < 0)
    S.at<double>(2, 2) = -1;
  Mat R_mat = U*S*Vt;
  //cout << R_mat << endl;
  double c2 = 0;
  c2 = (D.at<double>(0) + D.at<double>(1) + D.at<double>(2))/sigmx2;
  //cout << c2 << endl;
  t[0] = meany[0] - c2*(R_mat.at<double>(0,0)*meanx[0]
                        + R_mat.at<double>(0,1)*meanx[1]
                        + R_mat.at<double>(0,2)*meanx[2]);
  t[1] = meany[1] - c2*(R_mat.at<double>(1,0)*meanx[0]
                        + R_mat.at<double>(1,1)*meanx[1]
                        + R_mat.at<double>(1,2)*meanx[2]);
  t[2] = meany[2] - c2*(R_mat.at<double>(2,0)*meanx[0]
                        + R_mat.at<double>(2,1)*meanx[1]
                        + R_mat.at<double>(2,2)*meanx[2]);
  //cout << t[0] << " " << t[1] << " " << t[2] << endl;

  double r1[3], r2[3], r3[3], r12[3];
  r1[0] = R_mat.at<double>(0,0);  r2[0] = R_mat.at<double>(0,1);  r3[0] = R_mat.at<double>(0,2);
  r1[1] = R_mat.at<double>(1,0);  r2[1] = R_mat.at<double>(1,1);  r3[1] = R_mat.at<double>(1,2);
  r1[2] = R_mat.at<double>(2,0);  r2[2] = R_mat.at<double>(2,1);  r3[2] = R_mat.at<double>(2,2);
  xcross(r1, r2, r12);
  if (norm (r12[0] - r3[0], r12[1] - r3[1], r12[2] - r3[2]) > 2e-2) {
    R_mat.at<double>(0,2) = -r3[0];
    R_mat.at<double>(1,2) = -r3[1];
    R_mat.at<double>(2,2) = -r3[2];
  }
  //cout << R_mat << endl;
  R[0][0] = R_mat.at<double>(0,0);  R[0][1] = R_mat.at<double>(0,1);  R[0][2] = R_mat.at<double>(0,2);
  R[1][0] = R_mat.at<double>(1,0);  R[1][1] = R_mat.at<double>(1,1);  R[1][2] = R_mat.at<double>(1,2);
  R[2][0] = R_mat.at<double>(2,0);  R[2][1] = R_mat.at<double>(2,1);  R[2][2] = R_mat.at<double>(2,2);
}

void rpnp::polyval (double D6[7], double t2s[7], double F6[7]) {
  for (int i=0; i<7; ++i) {
    if (t2s[i] != 0)
      F6[i] = D6[0]*pow(t2s[i],6)
            + D6[1]*pow(t2s[i],5)
            + D6[2]*pow(t2s[i],4)
            + D6[3]*pow(t2s[i],3)
            + D6[4]*pow(t2s[i],2)
            + D6[5]*pow(t2s[i],1)
            + D6[6];
    else
      F6[i] = 0;
    //cout << F6[i] << endl;
  }
}

double rpnp::norm (double u, double v, double w) {
  return sqrt( u*u + v*v + w*w );
}

double rpnp::norm (double u[3]) {
  return sqrt( u[0]*u[0] + u[1]*u[1] + u[2]*u[2] );
}

void rpnp::xcross(double a[3], double b[3], double c[3]) {
  c[0] = a[1]*b[2]-a[2]*b[1];
  c[1] = a[2]*b[0]-a[0]*b[2];
  c[2] = a[0]*b[1]-a[1]*b[0];
}

void rpnp::roots (double D7[8], double real[7], double imag[7]) {
  Mat coefficients (1, 8, CV_64F);
  coefficients.at<double>(0,0) = D7[7];
  coefficients.at<double>(0,1) = D7[6];
  coefficients.at<double>(0,2) = D7[5];
  coefficients.at<double>(0,3) = D7[4];
  coefficients.at<double>(0,4) = D7[3];
  coefficients.at<double>(0,5) = D7[2];
  coefficients.at<double>(0,6) = D7[1];
  coefficients.at<double>(0,7) = D7[0];
  Mat roots_mat (7, 2, CV_64F);
  solvePoly (coefficients, roots_mat);
  real[0] = roots_mat.at<double>(0,0);
  real[1] = roots_mat.at<double>(1,0);
  real[2] = roots_mat.at<double>(2,0);
  real[3] = roots_mat.at<double>(3,0);
  real[4] = roots_mat.at<double>(4,0);
  real[5] = roots_mat.at<double>(5,0);
  real[6] = roots_mat.at<double>(6,0);
  imag[0] = roots_mat.at<double>(0,1);
  imag[1] = roots_mat.at<double>(1,1);
  imag[2] = roots_mat.at<double>(2,1);
  imag[3] = roots_mat.at<double>(3,1);
  imag[4] = roots_mat.at<double>(4,1);
  imag[5] = roots_mat.at<double>(5,1);
  imag[6] = roots_mat.at<double>(6,1);
  //cout << t2s[0] << " " << t2s[1] << " " << t2s[2] << " " << t2s[3] << " " << t2s[4] << " " << t2s[5] << " " << t2s[6] << " " << endl;
}

void rpnp::eig (double DTD[6][6], double V[6][6], double D[6][6]) {
  Mat DTD_mat (6, 6, CV_64F, DTD);
  Mat eigenvalues, eigenvectors;
  eigen(DTD_mat , eigenvalues, eigenvectors);
  D[0][0] = eigenvalues.at<double>(5);  D[1][1] = eigenvalues.at<double>(4);
  D[3][3] = eigenvalues.at<double>(2);  D[2][2] = eigenvalues.at<double>(3);
  D[5][5] = eigenvalues.at<double>(0);  D[4][4] = eigenvalues.at<double>(1);
  for(int i=0; i<6; ++i)
    for(int j=0; j<6; ++j)
      V[i][j] = eigenvectors.at<double>(5-j, i);
  /*
  for(int i=0; i<6; ++i) {
    for(int j=0; j<6; ++j) {
      cout << V[i][j] << " ";
    }
    cout << endl;
  }
  */
}

double rpnp::max_array (double t2s[7]) {
  double tmp = 0.0;
  for (int i=0; i<7; ++i)
    tmp = max(tmp, fabs(t2s[i]));
  return tmp;
}

void rpnp::mat_to_quat(const double R[3][3], double q[4]) {
  double tr = R[0][0] + R[1][1] + R[2][2];
  double n4;

  if (tr > 0.0f) {
    q[0] = R[1][2] - R[2][1];
    q[1] = R[2][0] - R[0][2];
    q[2] = R[0][1] - R[1][0];
    q[3] = tr + 1.0f;
    n4 = q[3];
  } else if ( (R[0][0] > R[1][1]) && (R[0][0] > R[2][2]) ) {
    q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
    q[1] = R[1][0] + R[0][1];
    q[2] = R[2][0] + R[0][2];
    q[3] = R[1][2] - R[2][1];
    n4 = q[0];
  } else if (R[1][1] > R[2][2]) {
    q[0] = R[1][0] + R[0][1];
    q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
    q[2] = R[2][1] + R[1][2];
    q[3] = R[2][0] - R[0][2];
    n4 = q[1];
  } else {
    q[0] = R[2][0] + R[0][2];
    q[1] = R[2][1] + R[1][2];
    q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
    q[3] = R[0][1] - R[1][0];
    n4 = q[2];
  }
  double scale = 0.5f / double(sqrt(n4));

  q[0] *= scale;
  q[1] *= scale;
  q[2] *= scale;
  q[3] *= scale;
}

double rpnp::dot(const double * v1, const double * v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void rpnp::log (string msg) {
  ofstream file;
  file.open("rpnp_log.log", ios_base::app);
  //cout << msg << endl;
  if (file.is_open())
    file << msg << endl;
  file.close ();
}
