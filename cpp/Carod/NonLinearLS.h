#ifndef _TYPES_G2O_PTAM_
#define _TYPES_G2O_PTAM_

#include <Eigen/StdVector>
#include <tr1/random>
#include <iostream>
#include <stdint.h>
#include <tr1/unordered_set>

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/structure_only_solver.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <cstdio>
#include <iostream>
#include <fstream>

using namespace std;
//using namespace Eigen;


typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 4, 1> Vector4;
typedef Eigen::Matrix<double, 5, 1> Vector5;
typedef Eigen::Matrix<double, 6, 1> Vector6;

typedef Eigen::Matrix<double, 3, 3> Matrix3;
typedef Eigen::Matrix<double, 4, 4> Matrix4;
typedef Eigen::Matrix<double, 6, 6> Matrix6;



class SE3AxisAngle
{
public:
//  enum { Dim = 6 };

  SE3AxisAngle()
  {
     t.fill(0);
     axis.fill(0);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline SE3AxisAngle(const Vector6 mu)
  {
     t = mu.segment<3>(0);
     axis = mu.segment<3>(3);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void setAxisAngle(const Vector6  mu)
  {
     t = mu.segment<3>(0);
     axis = mu.segment<3>(3);
  }

  //////////////////////////////////////////////////////////////////////////////

  SE3AxisAngle& operator=(const Matrix4& m)
  {
     Vector6 v6;
     v6 = log(m);
     t = v6.segment<3>(0);
     axis = v6.segment<3>(3);
     return *this;
  }

  //////////////////////////////////////////////////////////////////////////////

  Vector3 map(const Vector3& pto_xyz)
  {
     Matrix4 T;
     T = exp(getAxisAngle());
     return (T.block<3,3>(0,0) *  pto_xyz) + T.block<3,1>(0,3);
  }

  //////////////////////////////////////////////////////////////////////////////

  Vector3 mapInv(const Vector3& pto_xyz)
  {
     Matrix4 T;
     T = exp(-getAxisAngle());
     return (T.block<3,3>(0,0) *  pto_xyz) + T.block<3,1>(0,3);
  }

  //////////////////////////////////////////////////////////////////////////////

  Vector6 getAxisAngle(void)
  {
     Vector6 mu;
     mu.segment<3>(0) = t;
     mu.segment<3>(3) = axis;
     return(mu);
  }

  //////////////////////////////////////////////////////////////////////////////

  SE3AxisAngle inverse() { return SE3AxisAngle(log(exp(-getAxisAngle()))); }


  static Vector6 log(const Matrix4& m);
  static Matrix4 exp(const Vector6 & mu);
//  static Matrix4 expInv(const Vector6 & mu);

private:
  Vector3 t, axis;
};



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////



namespace g2o
{

  using namespace Eigen;

  /**
   * \brief Point vertex, XYZ
   */
  class VertexPointXYZ : public BaseVertex<3, Vector3>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      VertexPointXYZ()  {}

      //////////////////////////////////////////////////////////////////////////

      virtual bool read(std::istream& is)  { return true; }
      virtual bool write(std::ostream& os) const  { return true; }

      //////////////////////////////////////////////////////////////////////////

      virtual void setToOrigin() { _estimate.fill(0.); }

      //////////////////////////////////////////////////////////////////////////

      virtual void oplus(double* update_)
      {
         //Vector3 update;
         for (int i=0; i<3; i++)   _estimate[i] += update_[i];
         //_estimate += update;
      }
  };


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
  /**
   * Aligment transformation
   */
  class VertexSE3AxisAngle : public BaseVertex<6, SE3AxisAngle>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      VertexSE3AxisAngle(Vector6 calibracion): K(calibracion)
      {
         Rcw = MatrixXd::Identity(3, 3);
         tcw = MatrixXd::Zero(3, 1);
      }

      //////////////////////////////////////////////////////////////////////////

      VertexSE3AxisAngle(Vector6 calibracion, Vector6 mu, bool Twc = false)
      : K(calibracion)
      {
         Matrix4 upd;
         upd = SE3AxisAngle::exp(mu);

         this->Twc = Twc;
         if(Twc)
         {
            Rcw = (upd.block<3,3>(0,0)).transpose();
            tcw = -Rcw * upd.block<3,1>(0,3);
         }else
         {
            Rcw = upd.block<3,3>(0,0);
            tcw = upd.block<3,1>(0,3);
         }
      }

      //////////////////////////////////////////////////////////////////////////

      VertexSE3AxisAngle(Vector6 calibracion, Matrix4 T, bool Twc = false)
      : K(calibracion)
      {
         this->Twc = Twc;
         if(Twc)
         {
            Rcw = (T.block<3,3>(0,0)).transpose();
            tcw = -Rcw * T.block<3,1>(0,3);
         }else
         {
            Rcw = T.block<3,3>(0,0);
            tcw = T.block<3,1>(0,3);
         }
      }
      //////////////////////////////////////////////////////////////////////////

      bool read(std::istream& is)  { return true; }
      bool write(std::ostream& os) const  { return true; }

      //////////////////////////////////////////////////////////////////////////

      virtual void setToOrigin()  { _estimate = SE3AxisAngle(); }

      //////////////////////////////////////////////////////////////////////////

      virtual void oplus(double* update_)
      {
         Vector6 update;
         Matrix4 upd;
         for (int i=0; i<6; i++)   update[i] = update_[i];

         // Tcw = exp(update) * Tcw
         upd = SE3AxisAngle::exp(update);
         Rcw = upd.block<3,3>(0,0) * Rcw;
         tcw = upd.block<3,3>(0,0) * tcw + upd.block<3,1>(0,3);
      }

      //////////////////////////////////////////////////////////////////////////

      Vector2d map(const Vector3 & Pw)
      {
        double fx, fy, cx, cy, k1, k2;
        cx = K(0);
        cy = K(1);
        fx = K(2);
        fy = K(3);
        k1 = K(4);
        k2 = K(5);

        double r2, r4, L;
        // Pc = Tcw * Pw
        Vector3 Pc =Rcw * Pw + tcw;
        Vector2d proj;    proj(0) = Pc(0)/Pc(2);      proj(1) = Pc(1)/Pc(2);
        r2 = proj[0]*proj[0] + proj[1]*proj[1];
        r4 = r2*r2;
        L  = 1 + k1*r2 + k2*r4;

        Vector2d res;
        res[0] = cx + fx * L * proj[0];
        res[1] = cy + fy * L * proj[1];
        return res;
      }

      //////////////////////////////////////////////////////////////////////////

      void push()
      {
         RtMatrix *upd;
         upd = new RtMatrix();
         upd->block<3,3>(0,0) = Rcw;
         upd->block<3,1>(0,3) = tcw;
         backups.push(upd);
         BaseVertex<6, SE3AxisAngle>::push();
      }

      //////////////////////////////////////////////////////////////////////////

      void pop()
      {
         RtMatrix *upd;
         assert(!backups.empty());
         upd = backups.top();
         Rcw = upd->block<3,3>(0,0);
         tcw = upd->block<3,1>(0,3);
         backups.pop();
         delete upd;
         BaseVertex<6, SE3AxisAngle>::pop();
      }

      //////////////////////////////////////////////////////////////////////////

      Matrix4 getCamera(bool Twc=false)
      {
         Vector6 cam;
         Matrix4 upd;
         upd.setIdentity();
         if(Twc)
         {
            upd.block<3,3>(0,0) = Rcw.transpose();
            upd.block<3,1>(0,3) = -upd.block<3,3>(0,0) * tcw;
         }else
         {
            upd.block<3,3>(0,0) = Rcw;
            upd.block<3,1>(0,3) = tcw;
         }
         //cam = SE3AxisAngle::log(upd);
         //return cam;
         return upd;
      }

    private:
      typedef Matrix<double, 3, 4> RtMatrix;

      std::stack<RtMatrix*> backups;

      // Tcw = [Rcw | tcw]
      bool Twc;
      Matrix3 Rcw;
      Vector3 tcw;
      Vector6 K;
  };

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

  // Edge declaration
  class EdgeProjectXYZ2UV : public  BaseMultiEdge<2, Vector2d>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      EdgeProjectXYZ2UV(){resize(2);};

      //////////////////////////////////////////////////////////////////////////

      bool read(std::istream& is){return true;};
      bool write(std::ostream& os) const{return true;};

      //////////////////////////////////////////////////////////////////////////

      void computeError()
      {
        VertexSE3AxisAngle*   cam =
           static_cast<VertexSE3AxisAngle*>(_vertices[0]);
//           static_cast<const VertexSE3AxisAngle*>(_vertices[0]);
        VertexPointXYZ* pto =
           static_cast<VertexPointXYZ*>(_vertices[1]);
//           static_cast<const VertexPointXYZ*>(_vertices[1]);

        Vector2d obs(_measurement);
        _error = obs - cam->map(pto->estimate());
      }

      //////////////////////////////////////////////////////////////////////////

//      virtual void linearizeOplus(); // Tengo que meter la jacobiana del error
                                       // de forma analitica. Ahora es numerica
  };


} // end namespace





// OPTIMIZATION
typedef struct
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector6 cal;
  Matrix4 cam;
  vector < Vector2, Eigen::aligned_allocator<Vector2> > p2d;
  vector < Vector3, Eigen::aligned_allocator<Vector3> > p3d;
  //Matrix4 solMatlab;
} OptimizationData;


void poseOptimization(OptimizationData &dat);

#endif
