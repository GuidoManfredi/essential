#include "NonLinearLS.h"

Matrix4 SE3AxisAngle::exp(const Vector6 & mu)
{
  Matrix3 w, w2, I;
  Matrix4 M;

  double A,B,C;
  double angle, angle2, angle4;

  w.setZero();
  w(0,1) = -mu[5];
  w(0,2) =  mu[4];
  w(1,0) =  mu[5];
  w(1,2) = -mu[3];
  w(2,0) = -mu[4];
  w(2,1) =  mu[3];

  w2 = w*w;

  angle2 = mu[3]*mu[3] + mu[4]*mu[4] + mu[5]*mu[5];
  angle  = sqrt(angle2);
  angle4 = angle2 * angle2;

  if (fabs(angle) < 1e-8)
  {
    A = 1 - (angle2)/6 + (angle4)/120;
    B = 0.5 - (angle2)/24 + (angle4)/720;
    C = 0;
  }else
  {
    A = sin(angle) / angle;
    B = (1 - cos(angle)) / angle2;
    C = (1-A) / angle2;
  }

  I.setIdentity();
  M.setIdentity();
  M.block<3,3>(0,0) =  I + w*A + w2*B;
  M.block<3,1>(0,3) = (I + w*B + w2*C) * mu.segment<3>(0);

  return M;
}

////////////////////////////////////////////////////////////////////////////////

Vector6 SE3AxisAngle::log(const Matrix4& m) /// Falta comprobar si angle es pi
{
  Vector6 v6;
  Vector3 axis;
  // cos(th) = (Tr(R)-1) / 2
  double C = (m(0,0)+m(1,1)+m(2,2)-1) * 0.5;
  double angle = acos(C);
  double angle2 = angle*angle;
  double S = sin(angle);
  double A,B;

  if(fabs(angle) < 1e-8)
  {
     A = 1 - (angle2)/6 + (angle2 * angle2)/120;
     B = 0;
  }else
  {
     A = sin(angle) / angle;
     B = (2*S - angle*(1+C)) / (2*angle2*S);
  }

  Matrix3 w = (m.block<3,3>(0,0) - m.block<3,3>(0,0).transpose()) / (2*A);
  Matrix3 I;
  I.setIdentity();

  axis(0) = w(2,1);   axis(1) = w(0,2);   axis(2) = w(1,0);

  v6.segment<3>(0) = (I - 0.5*w + B*w*w) * m.block<3,1>(0,3);
  v6.segment<3>(3) = axis;

  return v6;
}

////////////////////////////////////////////////////////////////////////////////

/*
Matrix4 SE3AxisAngle::expInv(const Vector6 & mu)
{
//  Matrix4 Md, Mi;
//  Md = exp(mu);
//  Mi.setIdentity();
//  Mi.block<3,3>(0,0) =  Md.block<3,3>(0,0).transpose();
//  Mi.block<3,1>(0,3) = -Mi.block<3,3>(0,0) * Md.block<3,1>(0,3);
//  return Mi;
    return exp(-mu);
}
*/

////////////////////////////////////////////////////////////////////////////////

/*void EdgeKProjectXYZ2UV::linearizeOplus()
{
  // Jacobianas analiticas
}*/





void poseOptimization(OptimizationData &dat)
{
  int ITERACIONES = 200;
// Configuracion del resolvedor

  g2o::SparseOptimizer optimizer;
  optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  optimizer.setVerbose(false);

  g2o::BlockSolverX::LinearSolverType * linearSolver;
//  if (DENSE)
//  {
//     cerr << "MUY DENSO\n";
     linearSolver =
       new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
//  }else
  //{
  // linearSolver =
   //    new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
       //new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
       //new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();
//  }


  g2o::BlockSolverX * solver_ptr =
     new g2o::BlockSolverX(&optimizer,linearSolver);


  optimizer.setSolver(solver_ptr);

/******************************************************************************/

  int vertex_id = 0;


//////////////////////////////
/// Aligment transformation ///
/////////////////////////////
  g2o::VertexSE3AxisAngle * v_se3 = new g2o::VertexSE3AxisAngle(dat.cal, dat.cam);

  v_se3->setId(vertex_id);

  optimizer.addVertex(v_se3);
  vertex_id++;


//////////////
/// POINTS ///
//////////////
  for (size_t i=0; i < dat.p3d.size(); ++i)
  {
    g2o::VertexPointXYZ * v_p = new g2o::VertexPointXYZ();

    v_p->setId(vertex_id);
    v_p->setMarginalized(true);
    v_p->estimate() = dat.p3d[i];

    v_p->setFixed(true);

    g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
    e->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_se3);
    e->vertices()[1] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);
    e->measurement() = dat.p2d[i];
    e->inverseMeasurement() = -dat.p2d[i];

    e->information() = Eigen::Matrix2d::Identity();
    e->setRobustKernel(false);
    e->setHuberWidth(1);

    optimizer.addEdge(e);
    optimizer.addVertex(v_p);
    vertex_id ++;
  }


////////////////////
/// OPTIMIZATION ///
////////////////////

  //cout << "\n ----------------------------------- \n\n";


  optimizer.initializeOptimization();
  optimizer.setVerbose(false);

  optimizer.optimize(ITERACIONES);

  dat.cam = v_se3->getCamera();
}

