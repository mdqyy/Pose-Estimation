/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

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

inline MatrixXd computePoseJacobian(MatrixXd mxdObjPts, VectorXd v6dPose)
{
  unsigned uinPts = mxdObjPts.cols();
  MatrixXd mxdJacobian(2*uinPts,6);

  double ax = v6dPose(0);
  double ay = v6dPose(1);
  double az = v6dPose(2);
  double tx = v6dPose(3);
  double ty = v6dPose(4);
  double tz = v6dPose(5);

  double cx = cos(ax);
  double sx = sin(ax);
  double cy = cos(ay);
  double sy = sin(ay);
  double cz = cos(az);
  double sz = sin(az);

  double U,V,W;
  double didx, didy, didz, didtx, didty, didtz;
  double djdx, djdy, djdz, djdtx, djdty, djdtz;

  for (unsigned i = 0; i < mxdObjPts.cols(); i++)
  {
    U = mxdObjPts(0,i);
    V = mxdObjPts(1,i);
    W = mxdObjPts(2,i);

    didx = (V*(sx*sz + cx*cz*sy) + W*(cx*sz - cz*sx*sy))/(tz - U*sy + W*cx*cy + V*cy*sx) -
        ((V*cx*cy - W*cy*sx)*(tx - V*(cx*sz - cz*sx*sy) + W*(sx*sz + cx*cz*sy) + U*cy*cz))/
        ((tz - U*sy + W*cx*cy + V*cy*sx)*(tz - U*sy + W*cx*cy + V*cy*sx));
    didy = (W*cx*cy*cz - U*cz*sy + V*cy*cz*sx)/(tz - U*sy + W*cx*cy + V*cy*sx) +
        ((U*cy + W*cx*sy + V*sx*sy)*(tx - V*(cx*sz - cz*sx*sy) + W*(sx*sz + cx*cz*sy) + U*cy*cz))/
        ((tz - U*sy + W*cx*cy + V*cy*sx)*(tz - U*sy + W*cx*cy + V*cy*sx));
    didz = -(V*(cx*cz + sx*sy*sz) - W*(cz*sx - cx*sy*sz) + U*cy*sz)/(tz - U*sy + W*cx*cy + V*cy*sx);
    didtx = 1/(tz - U*sy + W*cx*cy + V*cy*sx);
    didty = 0;
    didtz = -(tx - V*(cx*sz - cz*sx*sy) + W*(sx*sz + cx*cz*sy) + U*cy*cz)/
        ((tz - U*sy + W*cx*cy + V*cy*sx)*(tz - U*sy + W*cx*cy + V*cy*sx));

    djdx = - (V*(cz*sx - cx*sy*sz) + W*(cx*cz + sx*sy*sz))/(tz - U*sy + W*cx*cy + V*cy*sx) -
        ((V*cx*cy - W*cy*sx)*(ty + V*(cx*cz + sx*sy*sz) - W*(cz*sx - cx*sy*sz) + U*cy*sz))/
        ((tz - U*sy + W*cx*cy + V*cy*sx)*(tz - U*sy + W*cx*cy + V*cy*sx));
    djdy = (W*cx*cy*sz - U*sy*sz + V*cy*sx*sz)/(tz - U*sy + W*cx*cy + V*cy*sx) +
        ((U*cy + W*cx*sy + V*sx*sy)*(ty + V*(cx*cz + sx*sy*sz) - W*(cz*sx - cx*sy*sz) + U*cy*sz))/
        ((tz - U*sy + W*cx*cy + V*cy*sx)*(tz - U*sy + W*cx*cy + V*cy*sx));
    djdz = (W*(sx*sz + cx*cz*sy) - V*(cx*sz - cz*sx*sy) + U*cy*cz)/(tz - U*sy + W*cx*cy + V*cy*sx);
    djdtx = 0;
    djdty = 1/(tz - U*sy + W*cx*cy + V*cy*sx);
    djdtz = -(ty + V*(cx*cz + sx*sy*sz) - W*(cz*sx - cx*sy*sz) + U*cy*sz)/
        ((tz - U*sy + W*cx*cy + V*cy*sx)*(tz - U*sy + W*cx*cy + V*cy*sx));

    mxdJacobian(2*i,0) = didx;
    mxdJacobian(2*i+1,0) = djdx;
    mxdJacobian(2*i,1) = didy;
    mxdJacobian(2*i+1,1) = djdy;
    mxdJacobian(2*i,2) = didz;
    mxdJacobian(2*i+1,2) = djdz;
    mxdJacobian(2*i,3) = didtx;
    mxdJacobian(2*i+1,3) = djdtx;
    mxdJacobian(2*i,4) = didty;
    mxdJacobian(2*i+1,4) = djdty;
    mxdJacobian(2*i,5) = didtz;
    mxdJacobian(2*i+1,5) = djdtz;
  }

  return mxdJacobian;
}

inline MatrixXd computeHomographyJacobian(MatrixXd mxdObjPts, Matrix3d m3dHomography)
{
  unsigned uinPts = mxdObjPts.cols();
  MatrixXd mxdJacobian = MatrixXd::Zero(2*uinPts,9);

  double h11 = m3dHomography(0,0);
  double h12 = m3dHomography(0,1);
  double h13 = m3dHomography(0,2);
  double h21 = m3dHomography(1,0);
  double h22 = m3dHomography(1,1);
  double h23 = m3dHomography(1,2);
  double h31 = m3dHomography(2,0);
  double h32 = m3dHomography(2,1);
  double h33 = m3dHomography(2,2);

  double u,v;

  for (unsigned i = 0; i < uinPts; i++)
  {
    u = mxdObjPts(0,i);
    v = mxdObjPts(1,i);

    mxdJacobian(2*i,0) = u/(h31*u + h32*v + h33);
    mxdJacobian(2*i,1) = v/(h31*u + h32*v + h33);
    mxdJacobian(2*i,2) = 1/(h31*u + h32*v + h33);
    mxdJacobian(2*i+1,3) = u/(h31*u + h32*v + h33);
    mxdJacobian(2*i+1,4) = v/(h31*u + h32*v + h33);
    mxdJacobian(2*i+1,5) = 1/(h31*u + h32*v + h33);
    mxdJacobian(2*i,6) = -(u*(h11*u + h12*v + h13))/((h31*u + h32*v + h33)*(h31*u + h32*v + h33));
    mxdJacobian(2*i+1,6) = -(u*(h21*u + h22*v + h23))/((h31*u + h32*v + h33)*(h31*u + h32*v + h33));
    mxdJacobian(2*i,7) = -(v*(h11*u + h12*v + h13))/((h31*u + h32*v + h33)*(h31*u + h32*v + h33));
    mxdJacobian(2*i+1,7) = -(v*(h21*u + h22*v + h23))/((h31*u + h32*v + h33)*(h31*u + h32*v + h33));
    mxdJacobian(2*i,8) = -(h11*u + h12*v + h13)/((h31*u + h32*v + h33)*(h31*u + h32*v + h33));
    mxdJacobian(2*i+1,8) = -(h21*u + h22*v + h23)/((h31*u + h32*v + h33)*(h31*u + h32*v + h33));
  }

  return mxdJacobian;
}

inline VectorXd fProject(MatrixXd mxdObjPts, VectorXd v6dPose)
{
  pose pPose(v6dPose);
  MatrixXd mxdImgPts = pPose.getPoseMatrix() * mxdObjPts;

  for (unsigned i = 0; i < mxdObjPts.cols(); i++)
  {
    mxdImgPts(0,i) = mxdImgPts(0,i)/mxdImgPts(2,i);
    mxdImgPts(1,i) = mxdImgPts(1,i)/mxdImgPts(2,i);
  }

  mxdImgPts.conservativeResize(2,mxdObjPts.cols());
  return Map<VectorXd>(mxdImgPts.data(),2*mxdObjPts.cols());
}

pose LM(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, pose pInitPose, unsigned uiMaxIterations)
{
  if (r3ObjectPoints.getnCols() != r2ImagePoints.getnCols())
  {
    cout << "Error LM(): Point correspondence mismatch" << endl;
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
  mxdImagePoints.conservativeResize(2,uinPts);

  VectorXd vxdImageVec =  Map<VectorXd>(mxdImagePoints.data(),2*uinPts);

  unsigned uiIterations = 0;
  VectorXd v6dinitPose = pInitPose.getPoseVector();
  VectorXd v6ddPose = VectorXd::Zero(6);
  VectorXd v6dlmPose = VectorXd::Zero(6);
  VectorXd vxddist;
  VectorXd vxddistlm;
  VectorXd vxdProjected = VectorXd::Zero(2*uinPts);
  MatrixXd mxdJacobian = MatrixXd::Zero(2*uinPts,6);
  MatrixXd mxdHessian;
  MatrixXd mxdHessianlm;

  double dRpErrlm;
  double dthreshold = 0.0001;
  double dRpErr = 1e+52;
  double dlambda;

  while (uiIterations < uiMaxIterations)
  {
    v6dlmPose = v6dinitPose + v6ddPose;

    vxdProjected = fProject(mxdObjectPoints, v6dlmPose);
    vxddistlm = vxdProjected - vxdImageVec;
    dRpErrlm = sqrt(vxddistlm.dot(vxddistlm));

    if (dRpErrlm < 0.001)
      break;

    if (dRpErrlm <= dRpErr)
    {
      if (uiIterations > 0 && dRpErrlm == dRpErr)
      {
        v6dinitPose = v6dlmPose;
        dRpErr = dRpErrlm;
        vxddist = vxddistlm;
        break;
      }

      mxdJacobian = computePoseJacobian(mxdObjectPoints, v6dlmPose);
      mxdHessian = mxdJacobian.transpose() * mxdJacobian;

      v6dinitPose = v6dlmPose;
      dRpErr = dRpErrlm;
      vxddist = vxddistlm;

      if (uiIterations == 0)
        dlambda = 0.001 * mxdHessian.trace()/6;

      else
        dlambda = 0.1 * dlambda;
    }

    else
      dlambda = 10 * dlambda;

    mxdHessianlm = mxdHessian + (dlambda * MatrixXd::Identity(6,6));
    v6ddPose = -1 * mxdHessianlm.inverse() * (mxdJacobian.transpose() * vxddist);

    uiIterations++;
  }

  pose se3pose(v6dlmPose);
  return se3pose;
}

pose coplanarLM(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, pose pInitPose, unsigned uiMaxIterations)
{
  if (r3ObjectPoints.getnCols() != r2ImagePoints.getnCols())
  {
    cout << "Error coplanarLM(): Point correspondence mismatch" << endl;
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
  mxdImagePoints.conservativeResize(2,uinPts);

  VectorXd vxdImageVec =  Map<VectorXd>(mxdImagePoints.data(),2*uinPts);

  pose se3pose;
  return se3pose;
}
