/**
Copyright (c) 2013, LAAS-CNRS
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the LAAS-CNRS the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE LAAS-CNRS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE LAAS-CNRS AND CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Guido Manfredi, LAAS-CNRS, August 2013
*/
/// author : Guido Manfredi
/// mail : gmanfredi.mail@gmail.com

/// This file shows how to generate synthetic data and use rpnp on them.

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

#include "rpnp.h"

using namespace std;
using namespace cv;

const double uc = 320;
const double vc = 240;
const double fu = 800;
const double fv = 800;

Mat K = (Mat_<double>(3, 3) << fu, 0, uc,
                                0,  fv, vc,
                                0 , 0, 1);
const int n = 1000;
const double noise = 10;

double rand(double min, double max)
{
  return min + (max - min) * double(rand()) / RAND_MAX;
}

void random_pose(double R[3][3], double t[3])
{
  const double range = 1;

  double phi   = rand(0, range * 3.14159 * 2);
  double theta = rand(0, range * 3.14159);
  double psi   = rand(0, range * 3.14159 * 2);

  R[0][0] = cos(psi) * cos(phi) - cos(theta) * sin(phi) * sin(psi);
  R[0][1] = cos(psi) * sin(phi) + cos(theta) * cos(phi) * sin(psi);
  R[0][2] = sin(psi) * sin(theta);

  R[1][0] = -sin(psi) * cos(phi) - cos(theta) * sin(phi) * cos(psi);
  R[1][1] = -sin(psi) * sin(phi) + cos(theta) * cos(phi) * cos(psi);
  R[1][2] = cos(psi) * sin(theta);

  R[2][0] = sin(theta) * sin(phi);
  R[2][1] = -sin(theta) * cos(phi);
  R[2][2] = cos(theta);

  t[0] = 7.0f;
  t[1] = -50.0f;
  t[2] = 2000.0f;
}

void random_point(double & Xw, double & Yw, double & Zw)
{
  double theta = rand(0, 3.14159), phi = rand(0, 2 * 3.14159), R = rand(0, +2);

  Xw = sin(theta) * sin(phi) * R;
  Yw = -sin(theta) * cos(phi) * R;
  Zw =  cos(theta) * R;
}

void project_with_noise(double R[3][3], double t[3],
			double Xw, double Yw, double Zw,
			double & u, double & v)
{
  double Xc = R[0][0] * Xw + R[0][1] * Yw + R[0][2] * Zw + t[0];
  double Yc = R[1][0] * Xw + R[1][1] * Yw + R[1][2] * Zw + t[1];
  double Zc = R[2][0] * Xw + R[2][1] * Yw + R[2][2] * Zw + t[2];

  double nu = rand(-noise, +noise);
  double nv = rand(-noise, +noise);
  u = uc + fu * Xc / Zc + nu;
  v = vc + fv * Yc / Zc + nv;
}

int main(int argc, char ** argv)
{
  srand(time(0));

  double R_true[3][3], t_true[3];
  random_pose(R_true, t_true);

  Mat XX (n, 3, CV_64F);
  Mat xx (n, 2, CV_64F);
  for(int i = 0; i < n; i++) {
    double Xw, Yw, Zw, u, v;
    random_point(Xw, Yw, Zw);
    project_with_noise(R_true, t_true, Xw, Yw, Zw, u, v);
    XX.at<double>(i, 0) = Xw;    XX.at<double>(i, 1) = Yw;    XX.at<double>(i, 2) = Zw;
    xx.at<double>(i, 0) = u;    xx.at<double>(i, 1) = v;
  }
  rpnp PnP (K, XX, xx);

  Mat R_est (3, 3, CV_64F), t_est (3, 1, CV_64F);
  PnP.compute_pose(R_est, t_est);

  cout << R_est << endl << t_est << endl;
  cout << t_true[0] << " " << t_true[1] << " " << t_true[2] << endl;

  return 0;
}
