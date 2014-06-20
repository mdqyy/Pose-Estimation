/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

        Pose from Orthography and Scaling with ITerations (POSIT)
        Computes the pose (R t) from point correspondences

        Usage:
                        pose = POSIT(object, image, camera, initPose, maxIters)[1]
                        pose = coplanarPOSIT(object, image, camera, initPose, maxIters)[2]

        Input:
                        object  :    (4 x n) 3D homogeneous object points (n: no. of object points)
                        imgage  :    (3 x n) 2D homogeneous image points (n: no. of image points)
                        camera  :    (3 x 3) Camera initinsic matrix
                        initPose:    (3 x 4) Initial pose estimate
                        maxIters:    Maximum no. of iterations

        Output:
                        pose.R:      (3 x 3) Rotation matrix
                        pose.T:      (3 x 1) Translation vector

        Implementation of the algorithm described in:

        [1]             Daniel F. DeMenthon , Larry S. Davis,
                        "Recognition and Tracking of 3D Objects by 1D Search"
                        Proc. Image Understanding Workshop, Washington, DC,
                        pp. 653-659, 1993
                        
        [2]             D. Oberkampf, D. F. DeMenthon, and L. S. Davis,
                        “Iterative pose estimation using coplanar feature points,” 
                        Computer Vision and Image Understanding, 
                        vol. 63, no. 3, pp. 495–511, May 1996.

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

pose POSIT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, unsigned uiMaxIterations)
{
  pose se3Pose;

  if (r3ObjectPoints.getnCols() != r2ImagePoints.getnCols())
  {
    cout << "Error POSIT(): Point correspondence mismatch" << endl;
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

  double dthreshold = 0.0001;
  double derror = 100;
  double dZ0;
  unsigned uiIterations = 0;

  VectorXd vxdui = mxdImagePoints.row(0);
  VectorXd vxdvi = mxdImagePoints.row(1);

  JacobiSVD<MatrixXd> svd;
  svd.compute(mxdObjectPoints, ComputeThinU | ComputeThinV);

  VectorXd vxdSingularValues = svd.singularValues();
  VectorXd vxdSingularValuesInv = VectorXd::Zero(uinPts);

  for (unsigned i = 0; i < uinPts; i++)
  {
    if (vxdSingularValues(i) >= 0.000001)
      vxdSingularValuesInv(i) = 1/vxdSingularValues(i);
  }

  MatrixXd mxdObjectMatrix = svd.matrixV() * vxdSingularValuesInv.asDiagonal() * svd.matrixU().transpose();

  Vector4d v4dvectI, v4dvectJ;
  Vector3d v3dunitI, v3dunitJ, v3dunitK;
  Vector3d v3dRotVec1, v3dRotVec2;
  Vector4d v4dPoseVec3;
  VectorXd vxdeps = VectorXd::Zero(uinPts);
  VectorXd vxdepsn = VectorXd::Zero(uinPts);

  double dscale, dscale1, dscale2;

  while(derror > dthreshold && uiIterations < uiMaxIterations)
  {
    v4dvectI = mxdObjectMatrix.transpose() * (vxdui.asDiagonal() * (VectorXd::Ones(uinPts) + vxdeps));
    v4dvectJ = mxdObjectMatrix.transpose() * (vxdvi.asDiagonal() * (VectorXd::Ones(uinPts) + vxdeps));

    Vector3d v3dvectI = v4dvectI.head(2);
    Vector3d v3dvectJ = v4dvectJ.head(2);

    dscale1 = v3dvectI.norm();
    dscale2 = v3dvectJ.norm();
    dscale = sqrt(dscale1 * dscale2);

    v3dunitI = v3dvectI/dscale1;
    v3dunitJ = v3dvectJ/dscale2;
    v3dunitK = v3dunitI.cross(v3dunitJ);

    dZ0 = 1/dscale;

    v4dPoseVec3.head(2) = v3dunitK;
    v4dPoseVec3(3) = dZ0;

    MatrixXd mxdObjectBlock = mxdObjectPoints.block(0,0,3,uinPts);
    vxdepsn = (mxdObjectBlock.transpose() * v3dunitK)/dZ0;

    VectorXd vxderror = vxdepsn - vxdeps;
    derror = vxderror.norm();
    vxdeps = vxdepsn;

    uiIterations += 1;
  }

  Matrix3d m3dR;
  Vector3d v3dt;

  m3dR.row(0) = v3dunitI;
  m3dR.row(1) = v3dunitJ;
  m3dR.row(2) = v3dunitK;

  v3dt(0) = v4dvectI(3)/dscale1;
  v3dt(1) = v4dvectJ(3)/dscale2;
  v3dt(2) = dZ0;

  se3Pose.setPose(m3dR,v3dt);

  return se3Pose;
}

inline double reProjErr(const MatrixXd mxdObjPts, const MatrixXd mxdImgPts, const Matrix3d m3dR, const Vector3d v3dt)
{
  MatrixXd mxdPose(3,4);
  mxdPose.block(0,0,3,3) = m3dR;
  mxdPose.block(0,3,3,1) = v3dt;

  MatrixXd mxdReProj = mxdPose * mxdObjPts;

  for (unsigned i = 0; i < mxdReProj.cols() ; i++)
  {
    mxdReProj(0,i) = mxdReProj(0,i)/mxdReProj(2,i);
    mxdReProj(1,i) = mxdReProj(1,i)/mxdReProj(2,i);
    mxdReProj(2,i) = mxdReProj(2,i)/mxdReProj(2,i);
  }

  return (mxdImgPts - mxdReProj).norm();
}

inline pose cPOSIT(MatrixXd mxdObjectPoints,  MatrixXd mxdObjectMatrix, MatrixXd mxdImagePoints, VectorXd vxdui, VectorXd vxdvi, Vector3d v3dvectU, unsigned uiMaxIterations, pose hypPose)
{
  pose se3Pose;

  unsigned uinPts = mxdObjectPoints.cols();

  double dthreshold = 0.000001;
  double derror = 100;
  unsigned uiIterations = 0;

  MatrixXd mxdObj = mxdObjectPoints.block(0,0,3,uinPts);

  Matrix3d m3dhypRot = hypPose.getRotationMatrix();
  Vector3d v3dhypTr = hypPose.getTranstalion();
  Vector3d v3dhypRot3 = m3dhypRot.row(2);

  double dZ0 = v3dhypTr(2);
  VectorXd vxdepsn = VectorXd::Zero(uinPts);
  VectorXd vxdeps = (1/dZ0)* mxdObj.transpose() * v3dhypRot3;

  Vector4d v4dvectI, v4dvectJ;
  Vector3d v3dvectI0, v3dvectJ0;
  Vector3d v3dvectIa, v3dvectJa, v3dvectIb, v3dvectJb;
  Vector3d v3dRotVeca1, v3dRotVeca2, v3dRotVeca3, v3dRotVecb1, v3dRotVecb2, v3dRotVecb3;
  Vector3d v3dRotVec1, v4dRotVec2, v4dRotVec3;
  VectorXd vxdzi1 = VectorXd::Zero(uinPts);
  VectorXd vxdzi2 = VectorXd::Zero(uinPts);
  Matrix3d m3dRot, m3dRota, m3dRotb;
  Vector3d v3dTr;
  pose se3posea, se3poseb;

  double dI0I0, dJ0J0, dI0J0;
  double ddelta, dq, dlambda, dmju;
  double dsa1, dsb1, dsa2, dsb2, dsa, dsb;
  double derra, derrb;
  double dtxa, dtya, dZ0a;
  double dtxb, dtyb, dZ0b;

  while(derror > dthreshold && uiIterations < uiMaxIterations)
  {
    v4dvectI = mxdObjectMatrix.transpose() * (vxdui.asDiagonal() * (VectorXd::Ones(uinPts) + vxdeps));
    v4dvectJ = mxdObjectMatrix.transpose() * (vxdvi.asDiagonal() * (VectorXd::Ones(uinPts) + vxdeps));

    v3dvectI0 = v4dvectI.head(2);
    v3dvectJ0 = v4dvectJ.head(2);

    dI0I0 = v3dvectI0.dot(v3dvectI0);
    dJ0J0 = v3dvectJ0.dot(v3dvectJ0);
    dI0J0 = v3dvectI0.dot(v3dvectJ0);

    ddelta = (dJ0J0-dI0I0)*(dJ0J0-dI0I0)+4*(dI0J0*dI0J0);

    if ((dI0I0 - dJ0J0) >= 0)
      dq = -1*(dI0I0-dJ0J0 + sqrt(ddelta))/2;

    else
      dq = -1*(dI0I0-dJ0J0 - sqrt(ddelta))/2;

    if (dq >= 0)
    {
      dlambda = sqrt(dq);
      if (dlambda == 0)
        dmju = 0;

      else
        dmju = -1*dI0J0/sqrt(dq);
    }

    else
    {
      dlambda = sqrt(-1*(dI0J0*dI0J0)/dq);
      if (dlambda == 0)
        dmju = sqrt(dI0I0-dJ0J0);

      else
        dmju = -1*dI0J0/sqrt(-1*(dI0J0*dI0J0)/dq);
    }

    v3dvectIa = v3dvectI0 + dlambda * v3dvectU;
    v3dvectJa = v3dvectJ0 + dmju * v3dvectU;

    dsa1 = v3dvectIa.norm();
    dsa2 = v3dvectJa.norm();
    dsa = sqrt(dsa1*dsa2);

    v3dRotVeca1 = v3dvectIa/dsa1;
    v3dRotVeca2 = v3dvectJa/dsa2;
    v3dRotVeca3 = v3dRotVeca1.cross(v3dRotVeca2);

    v3dvectIb = v3dvectI0 - dlambda * v3dvectU;
    v3dvectJb = v3dvectJ0 - dmju * v3dvectU;

    dsb1 = v3dvectIb.norm();
    dsb2 = v3dvectJb.norm();
    dsb = sqrt(dsb1*dsb2);

    v3dRotVecb1 = v3dvectIb/dsb;
    v3dRotVecb2 = v3dvectJb/dsb;
    v3dRotVecb3 = v3dRotVecb1.cross(v3dRotVecb2);

    dtxa = v4dvectI(3)/dsa;
    dtya = v4dvectJ(3)/dsa;
    dZ0a = 1/dsa;

    dtxb = v4dvectI(3)/dsb;
    dtyb = v4dvectJ(3)/dsb;
    dZ0b = 1/dsb;

    m3dRota.row(0) = v3dRotVeca1;
    m3dRota.row(1) = v3dRotVeca2;
    m3dRota.row(2) = v3dRotVeca3;

    se3posea.setPose(m3dRota,Vector3d(dtxa,dtya,dZ0a));

    m3dRotb.row(0) = v3dRotVecb1;
    m3dRotb.row(1) = v3dRotVecb2;
    m3dRotb.row(2) = v3dRotVecb3;

    se3poseb.setPose(m3dRotb,Vector3d(dtxb,dtyb,dZ0b));

    for (unsigned int i=0; i < uinPts; i++)
    {
      Vector3d v3dObjPts;
      v3dObjPts(0) = mxdObjectPoints.col(i)(0);
      v3dObjPts(1) = mxdObjectPoints.col(i)(1);
      v3dObjPts(2) = mxdObjectPoints.col(i)(2);

      vxdzi1(i) = dZ0a + v3dObjPts.dot(v3dRotVeca3);
      vxdzi2(i) = dZ0b + v3dObjPts.dot(v3dRotVecb3);
    }

    if (vxdzi1.minCoeff() < 0 && vxdzi2.minCoeff() > 0)
    {
      vxdepsn = (1/dZ0b)* mxdObj.transpose() * v3dRotVecb3;
      m3dRot = m3dRotb;
      v3dTr(0) = dtxb;
      v3dTr(1) = dtyb;
      v3dTr(2) = dZ0b;
    }

    else if (vxdzi2.minCoeff() < 0 && vxdzi1.minCoeff() > 0)
    {
      vxdepsn = (1/dZ0)* mxdObj.transpose() * v3dRotVeca3;
      m3dRot = m3dRota;
      v3dTr(0) = dtxa;
      v3dTr(1) = dtya;
      v3dTr(2) = dZ0a;
    }

    else if (vxdzi1.minCoeff() > 0 && vxdzi2.minCoeff() > 0)
    {
      derra = reProjErr(mxdObjectPoints, mxdImagePoints, m3dRota, Vector3d(dtxa,dtya,dZ0a));
      derrb = reProjErr(mxdObjectPoints, mxdImagePoints, m3dRotb, Vector3d(dtxb,dtyb,dZ0b));

      if (derra < derrb)
      {
        m3dRot = m3dRota;
        v3dTr(0) = dtxa;
        v3dTr(1) = dtya;
        v3dTr(2) = dZ0a;

        vxdepsn = (1/dZ0a)* mxdObj.transpose() * v3dRotVeca3;
      }

      else if (derra > derrb)
      {
        m3dRot = m3dRotb;
        v3dTr(0) = dtxb;
        v3dTr(1) = dtyb;
        v3dTr(2) = dZ0b;

        vxdepsn = (1/dZ0b)* mxdObj.transpose() * v3dRotVecb3;
      }
    }

    VectorXd vxderror = vxdepsn - vxdeps;
    derror = vxderror.norm();
    vxdeps = vxdepsn;

    uiIterations += 1;
  }

  se3Pose.setPose(m3dRot,v3dTr);
  return se3Pose;
}

pose coplanarPOSIT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, unsigned uiMaxIterations)
{
  pose se3Pose;

  if (r3ObjectPoints.getnCols() != r2ImagePoints.getnCols())
  {
    cout << "Error coplanarPOSIT(): Point correspondence mismatch" << endl;
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

  VectorXd vxdui = mxdImagePoints.row(0);
  VectorXd vxdvi = mxdImagePoints.row(1);

  JacobiSVD<MatrixXd> svd;
  svd.compute(mxdObjectPoints, ComputeThinU | ComputeThinV);

  VectorXd vxdSingularValues = svd.singularValues();
  VectorXd vxdSingularValuesInv = VectorXd::Zero(uinPts);

  for (unsigned i = 0; i < uinPts; i++)
  {
    if (vxdSingularValues(i) >= 0.000001)
      vxdSingularValuesInv(i) = 1/vxdSingularValues(i);
  }

  MatrixXd mxdObjectMatrix = svd.matrixV() * vxdSingularValuesInv.asDiagonal() * svd.matrixU().transpose();

  Vector4d v4dvectI, v4dvectJ;
  Vector3d v3dvectI0, v3dvectJ0;
  Vector3d v3dvectIa, v3dvectJa, v3dvectIb, v3dvectJb;
  Vector3d v3dRotVeca1, v3dRotVeca2, v3dRotVeca3, v3dRotVecb1, v3dRotVecb2, v3dRotVecb3;
  Vector3d v3dRotVec1, v4dRotVec2, v4dRotVec3;
  VectorXd vxdeps = VectorXd::Zero(uinPts);
  VectorXd vxdepsn = VectorXd::Zero(uinPts);
  VectorXd vxdzi1 = VectorXd::Zero(uinPts);
  VectorXd vxdzi2 = VectorXd::Zero(uinPts);
  Matrix3d m3dRot, m3dRota, m3dRotb;
  Vector3d v3dTr;
  pose se3posea, se3poseb;

  double dI0I0, dJ0J0, dI0J0;
  double ddelta, dq, dlambda, dmju;
  double dsa1, dsb1, dsa2, dsb2, dsa, dsb;
  double derra, derrb;
  double dtxa, dtya, dZ0a;
  double dtxb, dtyb, dZ0b;

  MatrixXd mxdObj = mxdObjectPoints.block(0,0,3,uinPts);

  svd.compute(mxdObj.transpose(), ComputeThinV);
  Vector3d v3dvectU = svd.matrixV().col(svd.matrixV().cols()-1);

  v4dvectI = mxdObjectMatrix.transpose() * (vxdui.asDiagonal() * (VectorXd::Ones(uinPts) + vxdeps));
  v4dvectJ = mxdObjectMatrix.transpose() * (vxdvi.asDiagonal() * (VectorXd::Ones(uinPts) + vxdeps));

  v3dvectI0 = v4dvectI.head(2);
  v3dvectJ0 = v4dvectJ.head(2);

  dI0I0 = v3dvectI0.dot(v3dvectI0);
  dJ0J0 = v3dvectJ0.dot(v3dvectJ0);
  dI0J0 = v3dvectI0.dot(v3dvectJ0);

  ddelta = (dJ0J0-dI0I0)*(dJ0J0-dI0I0)+4*(dI0J0*dI0J0);

  if ((dI0I0 - dJ0J0) >= 0)
    dq = -1*(dI0I0-dJ0J0 + sqrt(ddelta))/2;

  else
    dq = -1*(dI0I0-dJ0J0 - sqrt(ddelta))/2;

  if (dq >= 0)
  {
    dlambda = sqrt(dq);
    if (dlambda == 0)
      dmju = 0;

    else
      dmju = -1*dI0J0/sqrt(dq);
  }

  else
  {
    dlambda = sqrt(-1*(dI0J0*dI0J0)/dq);
    if (dlambda == 0)
      dmju = sqrt(dI0I0-dJ0J0);

    else
      dmju = -1*dI0J0/sqrt(-1*(dI0J0*dI0J0)/dq);
  }

  v3dvectIa = v3dvectI0 + dlambda * v3dvectU;
  v3dvectJa = v3dvectJ0 + dmju * v3dvectU;

  dsa1 = v3dvectIa.norm();
  dsa2 = v3dvectJa.norm();
  dsa = sqrt(dsa1*dsa2);

  v3dRotVeca1 = v3dvectIa/dsa1;
  v3dRotVeca2 = v3dvectJa/dsa2;
  v3dRotVeca3 = v3dRotVeca1.cross(v3dRotVeca2);

  v3dvectIb = v3dvectI0 - dlambda * v3dvectU;
  v3dvectJb = v3dvectJ0 - dmju * v3dvectU;

  dsb1 = v3dvectIb.norm();
  dsb2 = v3dvectJb.norm();
  dsb = sqrt(dsb1*dsb2);

  v3dRotVecb1 = v3dvectIb/dsb;
  v3dRotVecb2 = v3dvectJb/dsb;
  v3dRotVecb3 = v3dRotVecb1.cross(v3dRotVecb2);

  dtxa = v4dvectI(3)/dsa;
  dtya = v4dvectJ(3)/dsa;
  dZ0a = 1/dsa;

  dtxb = v4dvectI(3)/dsb;
  dtyb = v4dvectJ(3)/dsb;
  dZ0b = 1/dsb;

  m3dRota.row(0) = v3dRotVeca1;
  m3dRota.row(1) = v3dRotVeca2;
  m3dRota.row(2) = v3dRotVeca3;

  se3posea.setPose(m3dRota,Vector3d(dtxa,dtya,dZ0a));

  m3dRotb.row(0) = v3dRotVecb1;
  m3dRotb.row(1) = v3dRotVecb2;
  m3dRotb.row(2) = v3dRotVecb3;

  se3poseb.setPose(m3dRotb,Vector3d(dtxb,dtyb,dZ0b));

  for (unsigned int i=0; i < uinPts; i++)
  {
    Vector3d v3dObjPts;
    v3dObjPts(0) = mxdObjectPoints.col(i)(0);
    v3dObjPts(1) = mxdObjectPoints.col(i)(1);
    v3dObjPts(2) = mxdObjectPoints.col(i)(2);

    vxdzi1(i) = dZ0a + v3dObjPts.dot(v3dRotVeca3);
    vxdzi2(i) = dZ0b + v3dObjPts.dot(v3dRotVecb3);
  }

  if (vxdzi1.minCoeff() < 0 && vxdzi2.minCoeff() > 0)
  {
    se3Pose = cPOSIT(mxdObjectPoints, mxdObjectMatrix, mxdImagePoints, vxdui, vxdvi, v3dvectU, uiMaxIterations, se3poseb);
  }

  else if (vxdzi2.minCoeff() < 0 && vxdzi1.minCoeff() > 0)
  {
    se3Pose = cPOSIT(mxdObjectPoints, mxdObjectMatrix, mxdImagePoints, vxdui, vxdvi, v3dvectU, uiMaxIterations, se3posea);
  }

  else if (vxdzi1.minCoeff() > 0 && vxdzi2.minCoeff() > 0)
  {
    pose se3Pose1 = cPOSIT(mxdObjectPoints, mxdObjectMatrix, mxdImagePoints, vxdui, vxdvi, v3dvectU, uiMaxIterations, se3posea);
    pose se3Pose2 = cPOSIT(mxdObjectPoints, mxdObjectMatrix, mxdImagePoints, vxdui, vxdvi, v3dvectU, uiMaxIterations, se3poseb);

    derra = reProjErr(mxdObjectPoints, mxdImagePoints, se3Pose1.getRotationMatrix(), se3Pose1.getTranstalion());
    derrb = reProjErr(mxdObjectPoints, mxdImagePoints, se3Pose2.getRotationMatrix(), se3Pose2.getTranstalion());

    if (derra < derrb)
    {
      se3Pose = se3Pose1;
    }

    else
    {
      se3Pose = se3Pose2;
    }
  }

  return se3Pose;
}
