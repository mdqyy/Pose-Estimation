/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#include <iostream>
#include <sys/time.h>
#include <vector>
#include "3rdParty/Eigen/Eigen"
#include "3rdParty/Eigen/SVD"
#include "Camera.hpp"
#include "Object.hpp"
#include "Image.hpp"
#include "Pose.hpp"

using namespace std;
using namespace Eigen;

pose OI(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, pose pInitPose, unsigned uiMaxIterations)
{
  pose se3Pose;

  double derror = 1e+06;
  double dthreshold = 1e-06;
  unsigned uiIterations = 0;

  if (r3ObjectPoints.getnCols() != r2ImagePoints.getnCols())
  {
    cout << "Error OI(): Point correspondence mismatch" << endl;
  }

  MatrixXd mxdObjectPoints = r3ObjectPoints.getObjectPoints();
  MatrixXd mxdImagePoints = r2ImagePoints.getImagePoints();
  Matrix3d m3dK = m3Camera.getIntrinsic();

  unsigned uinPts = mxdObjectPoints.cols();

  MatrixXd mxdCrtdImage = m3dK.inverse() * mxdImagePoints;
  MatrixXd mxdImage = mxdCrtdImage;
  MatrixXd mxdObject = mxdObjectPoints.block(0,0,3,uinPts);

  Vector3d sumqi = mxdImage.rowwise().sum();
  Vector3d sumpi = mxdObject.rowwise().sum();

  Vector3d centqi = sumqi/((double)uinPts);
  Vector3d centpi = sumpi/((double)uinPts);

  MatrixXd centrdqi = mxdImage - centqi * MatrixXd::Ones(1,uinPts);
  MatrixXd centrdpi = mxdObject - centpi * MatrixXd::Ones(1,uinPts);

  Matrix3d m3dsumVi = Matrix3d::Zero();

  vector<Matrix3d> vm3dV(uinPts);
  unsigned uidx = 0;

  for (vector<Matrix3d>::iterator it=vm3dV.begin(); it!=vm3dV.end(); ++it)
  {
    Vector3d v3dv = mxdImage.col(uidx);
    *it = (v3dv*v3dv.transpose())/(v3dv.norm()*v3dv.norm());
    m3dsumVi += *it;

    uidx += 1;
  }

  Matrix3d m3dRk;
  Vector3d v3dtk;

  if (pInitPose.getRotationEuler() == Vector3d::Zero())
  {
    Matrix3d m3dMk0 = centrdqi * centrdpi.transpose();

    JacobiSVD<MatrixXd> svd;
    svd.compute(m3dMk0, ComputeThinU | ComputeThinV);

    m3dRk = svd.matrixU() * Matrix3d::Identity() * svd.matrixV().transpose();

    if (m3dRk.determinant() < 0)
    {
      MatrixXd m3dU = -1*svd.matrixU();
      m3dU.row(2) = svd.matrixU().row(2);
      m3dU.col(2) = svd.matrixU().col(2);

      m3dRk = m3dU * Matrix3d::Identity() * -1*svd.matrixV().transpose();
    }
  }

  else
  {
    m3dRk = pInitPose.getRotationMatrix();
    v3dtk = pInitPose.getTranstalion();
  }

  while (derror > dthreshold && uiIterations < uiMaxIterations)
  {
    Vector3d v3dsumViRpi = Vector3d::Zero();

    uidx = 0;
    for (vector<Matrix3d>::iterator it=vm3dV.begin(); it!=vm3dV.end(); ++it)
    {
      v3dsumViRpi += *it * m3dRk * mxdObject.col(uidx);
      uidx += 1;
    }

    v3dtk = (1/(double)uinPts)*(Matrix3d::Identity() - (1/(double)uinPts)*m3dsumVi).inverse() * (v3dsumViRpi - Matrix3d::Identity()*m3dRk*sumpi);

    MatrixXd mxdqi = MatrixXd::Zero(3,uinPts);
    uidx = 0;

    for (vector<Matrix3d>::iterator it=vm3dV.begin(); it!=vm3dV.end(); ++it)
    {
      mxdqi.col(uidx) = *it * (m3dRk * mxdObject.col(uidx) + v3dtk);
      uidx += 1;
    }

    Vector3d v3dcentqi = mxdqi.rowwise().mean();
    MatrixXd mxdcentrdqi = mxdqi - v3dcentqi * MatrixXd::Ones(1,uinPts);

    Matrix3d m3dMk = mxdcentrdqi * centrdpi.transpose();

    JacobiSVD<MatrixXd> svd2;
    svd2.compute(m3dMk, ComputeThinU | ComputeThinV);

    Matrix3d m3dRk1 = svd2.matrixU() * Matrix3d::Identity() * svd2.matrixV().transpose();

    if (m3dRk1.determinant() < 0)
    {
      MatrixXd m3dU1 = -1*svd2.matrixU();
      m3dU1.row(2) = svd2.matrixU().row(2);
      m3dU1.col(2) = svd2.matrixU().col(2);

      m3dRk = m3dU1 * Matrix3d::Identity() * -1*svd2.matrixV().transpose();
    }

    derror = (m3dRk1 - m3dRk).norm();
    m3dRk = m3dRk1;

    uiIterations += 1;
  }

  se3Pose.setPose(m3dRk,v3dtk);

  return se3Pose;
}
