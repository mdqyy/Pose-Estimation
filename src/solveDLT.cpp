/*------------------------------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

        Direct Linear Transform (DLT)
        Computes the pose (R t) from point correspondences

        Usage:
                        pose = DLT(object, image, camera)[1][2]
                        pose = normalizedDLT(object, image, camera)[2]
                        pose = coplanarDLTnormalizedDLT(object, image, camera)[2]

        Input:
                        object  :    (4 x n) 3D homogeneous object points (n: no. of object points)
                        imgage  :    (3 x n) 2D homogeneous image points (n: no. of image points)
                        camera  :    (3 x 3) Camera initinsic matrix

        Output:
                        pose.R:      (3 x 3) Rotation matrix
                        pose.T:      (3 x 1) Translation vector

        Implementation of the algorithms described in:

        [1]             E. Trucco and A. Verri, "Introductory Techniques for 3-D
                        Computer Vision," pp. 132-134, 1998.

        [2]             Hartley, R., & Zisserman, A. (2000). Multiple view geometry
                        in computer vision. Cambridge: Cambridge University Press.

------------------------------------------------------------------------------------------------*/

#include <iostream>
#include <sys/time.h>
#include "3rdParty/Eigen/Eigen"
#include "3rdParty/Eigen/SVD"
#include "Camera.hpp"
#include "Object.hpp"
#include "Image.hpp"
#include "Pose.hpp"

using namespace std;
using namespace Eigen;

pose DLT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera)
{
  pose se3Pose;

  if (r3ObjectPoints.getnCols() != r2ImagePoints.getnCols())
  {
    cout << "Error DLT(): Point correspondence mismatch" << endl;
  }

  MatrixXd mxdObjectPoints = r3ObjectPoints.getObjectPoints();
  MatrixXd mxdImagePoints = r2ImagePoints.getImagePoints();
  Matrix3d m3dK = m3Camera.getIntrinsic();

  unsigned uinPts = mxdObjectPoints.cols();
  double dFocalX = m3dK(0,0);
  double dFocalY = m3dK(1,1);
  int iCenterX = m3dK(0,2);
  int iCenterY = m3dK(1,2);

  mxdImagePoints.row(0) = (mxdImagePoints.row(0) - iCenterX*mxdImagePoints.row(2))/dFocalX;
  mxdImagePoints.row(1) = (mxdImagePoints.row(1) - iCenterY*mxdImagePoints.row(2))/dFocalY;

  MatrixXd mxdA(2*uinPts,12);
  Matrix3d m3dR;
  Vector3d v3dt;

  double dx, dX, dy, dY, dZ, dscale;

  for (unsigned i = 0; i < uinPts; i++)
  {
    dx = mxdImagePoints(0,i);
    dX = mxdObjectPoints(0,i);
    dy = mxdImagePoints(1,i);
    dY = mxdObjectPoints(1,i);
    dZ = mxdObjectPoints(2,i);

    mxdA(2*i,0) = dX;
    mxdA(2*i,1) = dY;
    mxdA(2*i,2) = dZ;
    mxdA(2*i,3) = 0;
    mxdA(2*i,4) = 0;
    mxdA(2*i,5) = 0;
    mxdA(2*i,6) = -1*dx*dX;
    mxdA(2*i,7) = -1*dx*dY;
    mxdA(2*i,8) = -1*dx*dZ;
    mxdA(2*i,9) = 1;
    mxdA(2*i,10) = 0;
    mxdA(2*i,11) = -1*dx;

    mxdA(2*i+1,0) = 0;
    mxdA(2*i+1,1) = 0;
    mxdA(2*i+1,2) = 0;
    mxdA(2*i+1,3) = dX;
    mxdA(2*i+1,4) = dY;
    mxdA(2*i+1,5) = dZ;
    mxdA(2*i+1,6) = -1*dy*dX;
    mxdA(2*i+1,7) = -1*dy*dY;
    mxdA(2*i+1,8) = -1*dy*dZ;
    mxdA(2*i+1,9) = 0;
    mxdA(2*i+1,10) = 1;
    mxdA(2*i+1,11) = -1*dy;
  }

  JacobiSVD<MatrixXd> svd, svd2;
  svd.compute(mxdA, ComputeThinV);

  VectorXd vxdM(12);
  vxdM = svd.matrixV().col(svd.matrixV().cols()-1);

  m3dR(0,0) = vxdM(0);
  m3dR(0,1) = vxdM(1);
  m3dR(0,2) = vxdM(2);
  m3dR(1,0) = vxdM(3);
  m3dR(1,1) = vxdM(4);
  m3dR(1,2) = vxdM(5);
  m3dR(2,0) = vxdM(6);
  m3dR(2,1) = vxdM(7);
  m3dR(2,2) = vxdM(8);

  v3dt(0) = vxdM(9);
  v3dt(1) = vxdM(10);
  v3dt(2) = vxdM(11);

  svd2.compute(m3dR, ComputeThinU | ComputeThinV);
  m3dR = svd2.matrixU() * Matrix3d::Identity() * svd2.matrixV().transpose();

  if (m3dR.determinant() < 0)
  {
    m3dR = -1*m3dR;
    v3dt = -1*v3dt;
  }

  VectorXd vxdD = svd2.singularValues();
  dscale = vxdD.sum()/3;
  v3dt = v3dt/dscale;

  se3Pose.setPose(m3dR, v3dt);

  return se3Pose;
}

pose coplanarDLT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera)
{
  pose se3Pose;

  if (r3ObjectPoints.getnCols() != r2ImagePoints.getnCols())
  {
    cout << "Error coplanarDLT(): Point correspondence mismatch" << endl;
  }

  MatrixXd mxdObjectPoints = r3ObjectPoints.getObjectPoints();
  MatrixXd mxdImagePoints = r2ImagePoints.getImagePoints();
  Matrix3d m3dK = m3Camera.getIntrinsic();

  unsigned uinPts = mxdObjectPoints.cols();
  double dFocalX = m3dK(0,0);
  double dFocalY = m3dK(1,1);
  int iCenterX = m3dK(0,2);
  int iCenterY = m3dK(1,2);

  mxdImagePoints.row(0) = (mxdImagePoints.row(0) - iCenterX*mxdImagePoints.row(2))/dFocalX;
  mxdImagePoints.row(1) = (mxdImagePoints.row(1) - iCenterY*mxdImagePoints.row(2))/dFocalY;

  Vector2d v2dImageCentroid = mxdImagePoints.rowwise().mean();
  Vector2d v2dObjectCentroid = mxdObjectPoints.rowwise().mean();

  mxdImagePoints.row(0) = mxdImagePoints.row(0) - v2dImageCentroid(0)*VectorXd::Ones(uinPts);
  mxdImagePoints.row(1) = mxdImagePoints.row(1) - v2dImageCentroid(1)*VectorXd::Ones(uinPts);

  mxdObjectPoints.row(0) = mxdObjectPoints.row(0) - v2dObjectCentroid(0)*VectorXd::Ones(uinPts);
  mxdObjectPoints.row(1) = mxdObjectPoints.row(1) - v2dObjectCentroid(1)*VectorXd::Ones(uinPts);

  double dmeanImageDist =  (mxdImagePoints.colwise().hypotNorm()).mean();
  double dmeanObjectDist =  (mxdObjectPoints.colwise().hypotNorm()).mean();

  double dscaleImage = sqrt(2)/dmeanImageDist;
  double dscaleObject = sqrt(2)/dmeanObjectDist;

  mxdImagePoints = dscaleImage * mxdImagePoints;
  mxdObjectPoints = dscaleObject * mxdObjectPoints;

  Matrix3d m3dT1 = Matrix3d::Identity();
  m3dT1(0,0) = dscaleImage;
  m3dT1(0,2) = -1*dscaleImage*v2dImageCentroid(0);
  m3dT1(1,1) = dscaleImage;
  m3dT1(1,2) = -1*dscaleImage*v2dImageCentroid(1);

  Matrix3d m3dT2 = Matrix3d::Identity();
  m3dT2(0,0) = dscaleObject;
  m3dT2(0,2) = -1*dscaleObject*v2dObjectCentroid(0);
  m3dT2(1,1) = dscaleObject;
  m3dT2(1,2) = -1*dscaleObject*v2dObjectCentroid(1);

  MatrixXd mxdA(2*uinPts,9);
  Matrix3d m3dR;
  Vector3d v3dt;

  double dx, dX, dy, dY;

  for (unsigned i = 0; i < uinPts; i++)
  {
    dx = mxdImagePoints(0,i);
    dX = mxdObjectPoints(0,i);
    dy = mxdImagePoints(1,i);
    dY = mxdObjectPoints(1,i);

    mxdA.row(2*i)   << dX, dY, 1, 0, 0, 0, -1*dx*dX, -1*dx*dY, -1*dx;
    mxdA.row(2*i+1) << 0, 0, 0, dX, dY, 1, -1*dy*dX, -1*dy*dY, -1*dy;
  }

  JacobiSVD<MatrixXd> svd, svd2;
  svd.compute(mxdA, ComputeThinV);

  VectorXd vxdM(9);
  vxdM = svd.matrixV().col(8);

  Matrix3d m3dH;

  m3dH(0,0) = vxdM(0);
  m3dH(0,1) = vxdM(1);
  m3dH(0,2) = vxdM(2);
  m3dH(1,0) = vxdM(3);
  m3dH(1,1) = vxdM(4);
  m3dH(1,2) = vxdM(5);
  m3dH(2,0) = vxdM(6);
  m3dH(2,1) = vxdM(7);
  m3dH(2,2) = vxdM(8);

  Matrix3d m3dHun = m3dT1.inverse() * m3dH * m3dT2;

  if (m3dHun(2,2) < 0)
    m3dHun = -1*m3dHun;

  Vector3d v3dCol0 = m3dHun.col(0);
  Vector3d v3dCol1 = m3dHun.col(1);
  Vector3d v3dCol2 = m3dHun.col(2);

  double dlambda1 = 1/v3dCol0.norm();
  double dlambda2 = 1/v3dCol1.norm();
  double dlambda3 = (dlambda1+dlambda2)/2;

  Vector3d v3dR1 = dlambda1*v3dCol0;
  Vector3d v3dR2 = dlambda2*v3dCol1;
  Vector3d v3dR3 = v3dCol0.cross(v3dR2);

  Matrix3d m3dRt;
  m3dRt.col(0) = v3dR1;
  m3dRt.col(1) = v3dR2;
  m3dRt.col(2) = v3dR3;

  svd2.compute(m3dRt, ComputeThinU | ComputeThinV);
  m3dR = svd2.matrixU() * Matrix3d::Identity() * svd2.matrixV().transpose();
  v3dt = dlambda3*v3dCol2;

  if (m3dR.determinant() < 0)
  {
    m3dR = -1*m3dR;
    v3dt = -1*v3dt;
  }

  se3Pose.setPose(m3dR,v3dt);

  return se3Pose;
}

pose normalizedDLT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera)
{
  pose se3Pose;

  if (r3ObjectPoints.getnCols() != r2ImagePoints.getnCols())
  {
    cout << "Error normalizedDLT(): Point correspondence mismatch" << endl;
  }

  MatrixXd mxdObjectPoints = r3ObjectPoints.getObjectPoints();
  MatrixXd mxdImagePoints = r2ImagePoints.getImagePoints();
  Matrix3d m3dK = m3Camera.getIntrinsic();

  unsigned uinPts = mxdObjectPoints.cols();
  double dFocalX = m3dK(0,0);
  double dFocalY = m3dK(1,1);
  int iCenterX = m3dK(0,2);
  int iCenterY = m3dK(1,2);

  mxdImagePoints.row(0) = (mxdImagePoints.row(0) - iCenterX*mxdImagePoints.row(2))/dFocalX;
  mxdImagePoints.row(1) = (mxdImagePoints.row(1) - iCenterY*mxdImagePoints.row(2))/dFocalY;

  Vector3d v3dImageCentroid = mxdImagePoints.rowwise().mean();
  Vector4d v4dObjectCentroid = mxdObjectPoints.rowwise().mean();

  mxdImagePoints.row(0) = mxdImagePoints.row(0) - v3dImageCentroid(0)*VectorXd::Ones(uinPts);
  mxdImagePoints.row(1) = mxdImagePoints.row(1) - v3dImageCentroid(1)*VectorXd::Ones(uinPts);
  mxdImagePoints.row(2) = mxdImagePoints.row(2) - v3dImageCentroid(2)*VectorXd::Ones(uinPts);

  mxdObjectPoints.row(0) = mxdObjectPoints.row(0) - v4dObjectCentroid(0)*VectorXd::Ones(uinPts);
  mxdObjectPoints.row(1) = mxdObjectPoints.row(1) - v4dObjectCentroid(1)*VectorXd::Ones(uinPts);
  mxdObjectPoints.row(2) = mxdObjectPoints.row(2) - v4dObjectCentroid(2)*VectorXd::Ones(uinPts);
  mxdObjectPoints.row(3) = mxdObjectPoints.row(3) - v4dObjectCentroid(3)*VectorXd::Ones(uinPts);

  double dmeanImageDist =  (mxdImagePoints.colwise().hypotNorm()).mean();
  double dmeanObjectDist =  (mxdObjectPoints.colwise().hypotNorm()).mean();

  double dscaleImage = sqrt(2)/dmeanImageDist;
  double dscaleObject = sqrt(3)/dmeanObjectDist;

  mxdImagePoints = dscaleImage * mxdImagePoints;
  mxdObjectPoints = dscaleObject * mxdObjectPoints;

  Matrix3d m3dT1 = Matrix3d::Identity();
  m3dT1(0,0) = dscaleImage;
  m3dT1(0,2) = -1*dscaleImage*v3dImageCentroid(0);
  m3dT1(1,1) = dscaleImage;
  m3dT1(1,2) = -1*dscaleImage*v3dImageCentroid(1);

  Matrix4d m4dT2 = Matrix4d::Identity();
  m4dT2(0,0) = dscaleObject;
  m4dT2(0,3) = -1*dscaleObject*v4dObjectCentroid(0);
  m4dT2(1,1) = dscaleObject;
  m4dT2(1,3) = -1*dscaleObject*v4dObjectCentroid(1);
  m4dT2(2,2) = dscaleObject;
  m4dT2(2,3) = -1*dscaleObject*v4dObjectCentroid(2);

  MatrixXd mxdA(2*uinPts,12);
  Matrix3d m3dR;
  Vector3d v3dt;

  double dx, dX, dy, dY, dZ, dscale;

  for (unsigned i = 0; i < uinPts; i++)
  {
    dx = mxdImagePoints(0,i);
    dX = mxdObjectPoints(0,i);
    dy = mxdImagePoints(1,i);
    dY = mxdObjectPoints(1,i);
    dZ = mxdObjectPoints(2,i);

    mxdA(2*i,0) = dX;
    mxdA(2*i,1) = dY;
    mxdA(2*i,2) = dZ;
    mxdA(2*i,3) = 1;
    mxdA(2*i,4) = 0;
    mxdA(2*i,5) = 0;
    mxdA(2*i,6) = 0;
    mxdA(2*i,7) = 0;
    mxdA(2*i,8) = -1*dx*dX;
    mxdA(2*i,9) = -1*dx*dY;
    mxdA(2*i,10) = -1*dx*dZ;
    mxdA(2*i,11) = -1*dx;

    mxdA(2*i+1,0) = 0;
    mxdA(2*i+1,1) = 0;
    mxdA(2*i+1,2) = 0;
    mxdA(2*i+1,3) = 0;
    mxdA(2*i+1,4) = dX;
    mxdA(2*i+1,5) = dY;
    mxdA(2*i+1,6) = dZ;
    mxdA(2*i+1,7) = 1;
    mxdA(2*i+1,8) = -1*dy*dX;
    mxdA(2*i+1,9) = -1*dy*dY;
    mxdA(2*i+1,10) = -1*dy*dZ;
    mxdA(2*i+1,11) = -1*dy;
  }

  JacobiSVD<MatrixXd> svd, svd2;
  svd.compute(mxdA, ComputeThinV);

  VectorXd vxdM(12);
  vxdM = svd.matrixV().col(11);

  MatrixXd m34dP(3,4);

  m34dP(0,0) = vxdM(0);
  m34dP(0,1) = vxdM(1);
  m34dP(0,2) = vxdM(2);
  m34dP(0,3) = vxdM(3);
  m34dP(1,0) = vxdM(4);
  m34dP(1,1) = vxdM(5);
  m34dP(1,2) = vxdM(6);
  m34dP(1,3) = vxdM(7);
  m34dP(2,0) = vxdM(8);
  m34dP(2,1) = vxdM(9);
  m34dP(2,2) = vxdM(10);
  m34dP(2,3) = vxdM(11);

  MatrixXd m34dPun = m3dT1.inverse() * m34dP * m4dT2;

  m3dR(0,0) = m34dPun(0,0);
  m3dR(0,1) = m34dPun(0,1);
  m3dR(0,2) = m34dPun(0,2);
  m3dR(1,0) = m34dPun(1,0);
  m3dR(1,1) = m34dPun(1,1);
  m3dR(1,2) = m34dPun(1,2);
  m3dR(2,0) = m34dPun(2,0);
  m3dR(2,1) = m34dPun(2,1);
  m3dR(2,2) = m34dPun(2,2);

  v3dt(0) = m34dPun(0,3);
  v3dt(1) = m34dPun(1,3);
  v3dt(2) = m34dPun(2,3);

  svd2.compute(m3dR, ComputeThinU | ComputeThinV);
  m3dR = svd2.matrixU() * Matrix3d::Identity() * svd2.matrixV().transpose();

  if (m3dR.determinant() < 0)
  {
    m3dR = -1*m3dR;
    v3dt = -1*v3dt;
  }

  VectorXd vxdD = svd2.singularValues();
  dscale = vxdD.sum()/3;
  v3dt = v3dt/dscale;

  se3Pose.setPose(m3dR, v3dt);

  return se3Pose;
}
