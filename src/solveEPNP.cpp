/*------------------------------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

        E-Perspective-n-Point (EPnP)
        Computes the pose (R t) from point correspondences

        Usage:
                        pose = EPNP(object, image, camera)[1]

        Input:
                        object  :    (4 x n) 3D homogeneous object points (n: no. of object points)
                        imgage  :    (3 x n) 2D homogeneous image points (n: no. of image points)
                        camera  :    (3 x 3) Camera initinsic matrix

        Output:
                        pose.R:      (3 x 3) Rotation matrix
                        pose.T:      (3 x 1) Translation vector

        Implementation of the algorithms described in:

        [1]             V. Lepetit, F. Moreno-Noguer, and P. Fua. EPnP: An accurate
                        O(n) solution to the pnp problem. Int. J. Comput. Vision,
                        81:155-166, February 2009. 7

------------------------------------------------------------------------------------------------*/

#include <iostream>
#include <vector>
#include <sys/time.h>
#include "3rdParty/Eigen/Eigen"
#include "3rdParty/Eigen/SVD"
#include "Camera.hpp"
#include "Object.hpp"
#include "Image.hpp"
#include "Pose.hpp"

using namespace std;
using namespace Eigen;

inline MatrixXd computeM(MatrixXd mxdImage, MatrixXd mxdAlphas, Matrix3d m3dK)
{
  unsigned uinPts = mxdImage.cols();
  double dfu = m3dK(0,0);
  double dfv = m3dK(1,1);
  double du0 = m3dK(0,2);
  double dv0 = m3dK(1,2);

  MatrixXd mxdM = MatrixXd::Zero(2*uinPts,12);

  double da1, da2, da3, da4;
  double dui, dvi;

  for (unsigned i=0; i<uinPts; i++)
  {
    da1 = mxdAlphas(i,0);
    da2 = mxdAlphas(i,1);
    da3 = mxdAlphas(i,2);
    da4 = mxdAlphas(i,3);

    dui = mxdImage(0,i);
    dvi = mxdImage(1,i);

    mxdM(2*i,0) = da1*dfu;
    mxdM(2*i,2) = da1*(du0-dui);
    mxdM(2*i,3) = da2*dfu;
    mxdM(2*i,5) = da2*(du0-dui);
    mxdM(2*i,6) = da3*dfu;
    mxdM(2*i,8) = da3*(du0-dui);
    mxdM(2*i,9) = da4*dfu;
    mxdM(2*i,11) = da4*(du0-dui);

    mxdM(2*i+1,1) = da1*dfv;
    mxdM(2*i+1,2) = da1*(dv0-dvi);
    mxdM(2*i+1,4) = da2*dfv;
    mxdM(2*i+1,5) = da2*(dv0-dvi);
    mxdM(2*i+1,7) = da3*dfv;
    mxdM(2*i+1,8) = da3*(dv0-dvi);
    mxdM(2*i+1,10) = da4*dfv;
    mxdM(2*i+1,11) = da4*(dv0-dvi);
  }

  return mxdM;
}

pose absoluteOrientation(MatrixXd mxdCloud1, MatrixXd mxdCloud2)
{
  pose se3Pose;

  unsigned uinPts = mxdCloud1.cols();

  Vector3d v3dcentr1 = mxdCloud1.rowwise().mean();
  Vector3d v3dcentr2 = mxdCloud2.rowwise().mean();

  MatrixXd mxdcentrd1 = mxdCloud1 - v3dcentr1 * MatrixXd::Ones(1,uinPts);
  MatrixXd mxdcentrd2 = mxdCloud2 - v3dcentr2 * MatrixXd::Ones(1,uinPts);

  Matrix3d m3dMk = mxdcentrd1 * mxdcentrd2.transpose();

  JacobiSVD<MatrixXd> svd;
  svd.compute(m3dMk, ComputeThinU | ComputeThinV);

  Matrix3d m3dRk = svd.matrixU() * Matrix3d::Identity() * svd.matrixV().transpose();

  if (m3dRk.determinant() < 0)
  {
    m3dRk = -1*m3dRk;
  }

  Vector3d v3dTr = v3dcentr1 - m3dRk * v3dcentr2;
  se3Pose.setPose(m3dRk,v3dTr);

  return se3Pose;
}

inline MatrixXd computeCameraCoords(VectorXd vxdX1, MatrixXd mxdCw, MatrixXd mxdAlpha, MatrixXd mxdXw)
{
    unsigned uinPts = mxdXw.cols();

    MatrixXd mxdCc_ = MatrixXd::Zero(4,3);
    mxdCc_.row(0) = vxdX1.segment(0,3);
    mxdCc_.row(1) = vxdX1.segment(3,3);
    mxdCc_.row(2) = vxdX1.segment(6,3);
    mxdCc_.row(3) = vxdX1.segment(9,3);

    MatrixXd mxdXc_ = mxdAlpha * mxdCc_;

    Vector3d v3dcentrXw = mxdXw.rowwise().mean();
    MatrixXd mxdcentrdXw = mxdXw - v3dcentrXw * MatrixXd::Ones(1,uinPts);
    VectorXd v3ddistw = mxdcentrdXw.cwiseAbs2().colwise().sum();
    v3ddistw = v3ddistw.cwiseSqrt();

    Vector3d v3dcentrXc = mxdXc_.colwise().mean();
    MatrixXd mxdcentrdXc = mxdXc_ - (v3dcentrXc * MatrixXd::Ones(1,uinPts)).transpose();
    VectorXd v3ddistc = mxdcentrdXc.cwiseAbs2().rowwise().sum();
    v3ddistc = v3ddistc.cwiseSqrt();

    double dsc = 1/((v3ddistc.transpose()*v3ddistc).inverse() * v3ddistc.transpose()*v3ddistw);
    MatrixXd mxdCc = mxdCc_/dsc;
    MatrixXd mxdXc = mxdAlpha * mxdCc;

    if (mxdXc.col(2).minCoeff() < 0)
      mxdXc = -1*mxdXc;

    return mxdXc;
}

inline MatrixXd compute_constraint_distance_2param_6eq_3unk(VectorXd vxdKm1, VectorXd vxdKm2)
{
  MatrixXd m63dP = MatrixXd::Zero(6,3);

  double m1_1 = vxdKm1(0);
  double m1_2 = vxdKm1(1);
  double m1_3 = vxdKm1(2);
  double m1_4 = vxdKm1(3);
  double m1_5 = vxdKm1(4);
  double m1_6 = vxdKm1(5);
  double m1_7 = vxdKm1(6);
  double m1_8 = vxdKm1(7);
  double m1_9 = vxdKm1(8);
  double m1_10 = vxdKm1(9);
  double m1_11 = vxdKm1(10);
  double m1_12 = vxdKm1(11);

  double m2_1 = vxdKm2(0);
  double m2_2 = vxdKm2(1);
  double m2_3 = vxdKm2(2);
  double m2_4 = vxdKm2(3);
  double m2_5 = vxdKm2(4);
  double m2_6 = vxdKm2(5);
  double m2_7 = vxdKm2(6);
  double m2_8 = vxdKm2(7);
  double m2_9 = vxdKm2(8);
  double m2_10 = vxdKm2(9);
  double m2_11 = vxdKm2(10);
  double m2_12 = vxdKm2(11);

  double t7 = m1_6 * m1_6;
  double t8 = m1_4 * m1_4;
  double t9 = m1_1 * m1_1;
  double t10 = m1_5 * m1_5;
  double t11 = m1_2 * m1_2;
  double t12 = m1_3 * m1_3;
  double t17 = m1_4 * m2_4;
  double t18 = m1_1 * m2_1;
  double t19 = m1_5 * m2_5;
  double t22 = m1_2 * m2_2;
  double t23 = m1_6 * m2_6;
  double t25 = m1_3 * m2_3;
  double t26 = (-1*m2_6 * m1_3 - m1_4 * m2_1 - m2_4 * m1_1 + t17 + t18 + t19 - m1_5 * m2_2 - m2_5 * m1_2 + t22 + t23 - m1_6 * m2_3 + t25);
  double t29 = m2_3 * m2_3;
  double t34 = m2_4 * m2_4;
  double t35 = m2_1 * m2_1;
  double t36 = m2_5 * m2_5;
  double t37 = m2_2 * m2_2;
  double t38 = m2_6 * m2_6;
  double t44 = m1_7 * m1_7;
  double t45 = m1_8 * m1_8;
  double t46 = m1_9 * m1_9;
  double t55 = m1_8 * m2_8;
  double t56 = m1_9 * m2_9;
  double t58 = m1_7 * m2_7;
  double t59 = (-1*m1_9 * m2_3 - m2_8 * m1_2 - m2_9 * m1_3 - m1_7 * m2_1 - m2_7 * m1_1 + t55 + t22 + t56 + t18 - m1_8 * m2_2 + t25 + t58);
  double t64 = m2_8 * m2_8;
  double t65 = m2_9 * m2_9;
  double t68 = m2_7 * m2_7;
  double t72 = m1_11 * m1_11;
  double t73 = m1_12 * m1_12;
  double t74 = m1_10 * m1_10;
  double t85 = m1_10 * m2_10;
  double t86 = m1_11 * m2_11;
  double t88 = m1_12 * m2_12;
  double t89 = (-1*m1_10 * m2_1 - m2_10 * m1_1 - m1_12 * m2_3 - m2_11 * m1_2 - m1_11 * m2_2 + t18 + t22 + t25 + t85 + t86 - m2_12 * m1_3 + t88);
  double t92 = m2_11 * m2_11;
  double t95 = m2_12 * m2_12;
  double t96 = m2_10 * m2_10;
  double t113 = (-1*m1_9 * m2_6 - m2_9 * m1_6 + t55 + t23 + t17 + t56 + t58 - m1_7 * m2_4 - m2_7 * m1_4 - m1_8 * m2_5 - m2_8 * m1_5 + t19);
  double t134 = (-1*m1_10 * m2_4 - m2_10 * m1_4 + t88 + t23 + t17 + t85 + t86 - m1_11 * m2_5 - m2_11 * m1_5 - m1_12 * m2_6 - m2_12 * m1_6 + t19);
  double t155 = (t58 + t88 - m2_10 * m1_7 - m2_11 * m1_8 + t56 - m1_10 * m2_7 + t55 + t85 + t86 - m1_12 * m2_9 - m2_12 * m1_9 - m1_11 * m2_8);

  m63dP(0,0) = -2 * m1_4 * m1_1 - 2 * m1_5 * m1_2 - 2 * m1_6 * m1_3 + t7 + t8 + t9 + t10 + t11 + t12;
  m63dP(0,1) = 2 * t26;
  m63dP(0,2) = -2 * m2_6 * m2_3 + t29 - 2 * m2_4 * m2_1 - 2 * m2_5 * m2_2 + t34 + t35 + t36 + t37 + t38;
  m63dP(1,0) = -2 * m1_7 * m1_1 + t12 - 2 * m1_9 * m1_3 + t44 + t45 + t46 - 2 * m1_8 * m1_2 + t9 + t11;
  m63dP(1,1) = 2 * t59;
  m63dP(1,2) = -2 * m2_8 * m2_2 - 2 * m2_9 * m2_3 + t64 + t65 - 2 * m2_7 * m2_1 + t29 + t68 + t37 + t35;
  m63dP(2,0) = t9 - 2 * m1_12 * m1_3 + t72 + t73 + t74 + t12 - 2 * m1_11 * m1_2 - 2 * m1_10 * m1_1 + t11;
  m63dP(2,1) = 2 * t89;
  m63dP(2,2) = -2 * m2_11 * m2_2 + t37 + t92 - 2 * m2_10 * m2_1 + t95 + t29 + t96 - 2 * m2_12 * m2_3 + t35;
  m63dP(3,0) = -2 * m1_9 * m1_6 + t8 + t10 + t7 - 2 * m1_7 * m1_4 + t44 + t45 + t46 - 2 * m1_8 * m1_5;
  m63dP(3,1) = 2 * t113;
  m63dP(3,2) = -2 * m2_9 * m2_6 + t68 + t64 - 2 * m2_7 * m2_4 - 2 * m2_8 * m2_5 + t34 + t36 + t38 + t65;
  m63dP(4,0) = t73 + t8 + t10 + t7 - 2 * m1_10 * m1_4 - 2 * m1_11 * m1_5 - 2 * m1_12 * m1_6 + t74 + t72;
  m63dP(4,1) = 2 * t134;
  m63dP(4,2) = -2 * m2_12 * m2_6 + t96 + t92 - 2 * m2_10 * m2_4 - 2 * m2_11 * m2_5 + t34 + t36 + t38 + t95;
  m63dP(5,0) = t46 + t73 + t44 + t45 + t74 + t72 - 2 * m1_11 * m1_8 - 2 * m1_10 * m1_7 - 2 * m1_12 * m1_9;
  m63dP(5,1) = 2 * t155;
  m63dP(5,2) = t65 - 2 * m2_10 * m2_7 + t96 + t92 + t95 - 2 * m2_11 * m2_8 + t68 + t64 - 2 * m2_12 * m2_9;

  return m63dP;
}

inline MatrixXd compute_constraint_distance_3param_6eq_6unk(VectorXd vxdKm1, VectorXd vxdKm2, VectorXd vxdKm3)
{
  MatrixXd m66dP = MatrixXd::Zero(6,6);

  double m1_1 = vxdKm1(0);
  double m1_2 = vxdKm1(1);
  double m1_3 = vxdKm1(2);
  double m1_4 = vxdKm1(3);
  double m1_5 = vxdKm1(4);
  double m1_6 = vxdKm1(5);
  double m1_7 = vxdKm1(6);
  double m1_8 = vxdKm1(7);
  double m1_9 = vxdKm1(8);
  double m1_10 = vxdKm1(9);
  double m1_11 = vxdKm1(10);
  double m1_12 = vxdKm1(11);

  double m2_1 = vxdKm2(0);
  double m2_2 = vxdKm2(1);
  double m2_3 = vxdKm2(2);
  double m2_4 = vxdKm2(3);
  double m2_5 = vxdKm2(4);
  double m2_6 = vxdKm2(5);
  double m2_7 = vxdKm2(6);
  double m2_8 = vxdKm2(7);
  double m2_9 = vxdKm2(8);
  double m2_10 = vxdKm2(9);
  double m2_11 = vxdKm2(10);
  double m2_12 = vxdKm2(11);

  double m3_1 = vxdKm3(0);
  double m3_2 = vxdKm3(1);
  double m3_3 = vxdKm3(2);
  double m3_4 = vxdKm3(3);
  double m3_5 = vxdKm3(4);
  double m3_6 = vxdKm3(5);
  double m3_7 = vxdKm3(6);
  double m3_8 = vxdKm3(7);
  double m3_9 = vxdKm3(8);
  double m3_10 = vxdKm3(9);
  double m3_11 = vxdKm3(10);
  double m3_12 = vxdKm3(11);

  double t1 = (m1_2 * m1_2);
  double t4 = (m1_6 * m1_6);
  double t5 = (m1_3 * m1_3);
  double t6 = (m1_5 * m1_5);
  double t11 = (m1_4 * m1_4);
  double t12 = (m1_1 * m1_1);
  double t20 = m1_4 * m2_4;
  double t21 = m1_3 * m2_3;
  double t22 = m1_5 * m2_5;
  double t23 = m1_2 * m2_2;
  double t24 = m1_6 * m2_6;
  double t25 = m1_1 * m2_1;
  double t26 = (-m2_4 * m1_1 - m2_5 * m1_2 - m2_6 * m1_3 - m1_6 * m2_3 - m1_4 * m2_1 - m1_5 * m2_2 + t20 + t21 + t22 + t23 + t24 + t25);
  double t27 = m1_6 * m3_6;
  double t29 = m1_5 * m3_5;
  double t30 = m1_4 * m3_4;
  double t33 = m1_3 * m3_3;
  double t35 = m1_1 * m3_1;
  double t38 = m1_2 * m3_2;
  double t39 = (t27 - m1_6 * m3_3 + t29 + t30 - m1_4 * m3_1 - m3_6 * m1_3 + t33 - m3_5 * m1_2 + t35 - m3_4 * m1_1 - m1_5 * m3_2 + t38);
  double t40 = (m2_4 * m2_4);
  double t41 = (m2_2 * m2_2);
  double t42 = (m2_5 * m2_5);
  double t43 = (m2_1 * m2_1);
  double t44 = (m2_6 * m2_6);
  double t45 = (m2_3 * m2_3);
  double t53 = m2_4 * m3_4;
  double t56 = m2_5 * m3_5;
  double t57 = m2_2 * m3_2;
  double t60 = m2_1 * m3_1;
  double t62 = m2_6 * m3_6;
  double t63 = m2_3 * m3_3;
  double t65 = (t53 - m2_4 * m3_1 - m3_4 * m2_1 + t56 + t57 - m2_5 * m3_2 - m2_6 * m3_3 + t60 - m3_6 * m2_3 + t62 + t63 - m3_5 * m2_2);
  double t66 = (m3_5 * m3_5);
  double t69 = (m3_4 * m3_4);
  double t70 = (m3_3 * m3_3);
  double t71 = (m3_2 * m3_2);
  double t72 = (m3_6 * m3_6);
  double t75 = (m3_1 * m3_1);
  double t81 = (m1_9 * m1_9);
  double t82 = (m1_8 * m1_8);
  double t83 = (m1_7 * m1_7);
  double t90 = m1_8 * m2_8;
  double t91 = m1_7 * m2_7;
  double t93 = m1_9 * m2_9;
  double t98 = (-m1_9 * m2_3 + t21 + t90 + t23 + t91 + t25 - m2_7 * m1_1 + t93 - m2_9 * m1_3 - m1_8 * m2_2 - m1_7 * m2_1 - m2_8 * m1_2);
  double t101 = m1_9 * m3_9;
  double t102 = m1_7 * m3_7;
  double t106 = m1_8 * m3_8;
  double t108 = (-m3_8 * m1_2 + t38 + t33 - m3_9 * m1_3 + t101 + t102 - m1_7 * m3_1 - m1_9 * m3_3 + t35 - m3_7 * m1_1 + t106 - m1_8 * m3_2);
  double t113 = (m2_8 * m2_8);
  double t116 = (m2_9 * m2_9);
  double t117 = (m2_7 * m2_7);
  double t119 = m2_8 * m3_8;
  double t122 = m2_9 * m3_9;
  double t125 = m2_7 * m3_7;
  double t128 = (t119 - m2_7 * m3_1 - m3_8 * m2_2 + t57 + t63 + t60 + t122 - m2_8 * m3_2 - m3_7 * m2_1 + t125 - m3_9 * m2_3 - m2_9 * m3_3);
  double t129 = (m3_7 * m3_7);
  double t130 = (m3_8 * m2_8);
  double t133 = (m3_9 * m3_9);
  double t141 = (m1_10 * m1_10);
  double t142 = (m1_11 * m1_11);
  double t143 = (m1_12 * m1_12);
  double t151 = m1_10 * m2_10;
  double t152 = m1_12 * m2_12;
  double t154 = m1_11 * m2_11;
  double t158 = (-m2_10 * m1_1 - m2_12 * m1_3 + t151 + t23 + t25 + t152 + t21 - m1_10 * m2_1 + t154 - m1_11 * m2_2 - m2_11 * m1_2 - m1_12 * m2_3);
  double t160 = m1_12 * m3_12;
  double t164 = m1_10 * m3_10;
  double t165 = m1_11 * m3_11;
  double t168 = (-m3_10 * m1_1 + t160 - m3_11 * m1_2 + t38 + t33 - m1_12 * m3_3 - m3_12 * m1_3 + t164 + t35 + t165 - m1_11 * m3_2 - m1_10 * m3_1);
  double t169 = (m2_12 * m2_12);
  double t170 = (m2_10 * m2_10);
  double t171 = (m2_11 * m2_11);
  double t179 = m2_10 * m3_10;
  double t181 = m2_12 * m3_12;
  double t185 = m2_11 * m3_11;
  double t188 = (t57 + t60 + t179 - m2_10 * m3_1 + t181 - m2_12 * m3_3 - m3_12 * m2_3 - m3_10 * m2_1 + t63 + t185 - m2_11 * m3_2 - m3_11 * m2_2);
  double t191 = (m3_12 * m3_12);
  double t192 = (m3_10 * m3_10);
  double t193 = (m3_11 * m3_11);
  double t212 = (t22 + t91 + t93 - m1_7 * m2_4 + t20 - m2_7 * m1_4 + t90 - m2_8 * m1_5 - m1_8 * m2_5 - m1_9 * m2_6 + t24 - m2_9 * m1_6);
  double t219 = (t101 + t30 - m3_9 * m1_6 - m3_8 * m1_5 - m1_8 * m3_5 - m3_7 * m1_4 + t27 - m1_9 * m3_6 - m1_7 * m3_4 + t102 + t106 + t29);
  double t233 = (t53 - m2_8 * m3_5 - m2_9 * m3_6 - m2_7 * m3_4 + t125 + t122 - m3_8 * m2_5 + t62 + t119 + t56 - m3_7 * m2_4 - m3_9 * m2_6);
  double t254 = (-m2_11 * m1_5 + t154 + t151 - m1_12 * m2_6 + t20 - m1_10 * m2_4 - m2_12 * m1_6 - m2_10 * m1_4 - m1_11 * m2_5 + t152 + t24 + t22);
  double t261 = (t30 - m3_12 * m1_6 - m1_10 * m3_4 - m1_11 * m3_5 + t160 + t27 + t164 + t165 - m3_11 * m1_5 - m3_10 * m1_4 - m1_12 * m3_6 + t29);
  double t275 = (-m3_10 * m2_4 + t56 - m2_10 * m3_4 + t62 - m3_12 * m2_6 - m2_11 * m3_5 + t53 - m3_11 * m2_5 - m2_12 * m3_6 + t179 + t181 + t185);
  double t296 = (-m2_12 * m1_9 + t152 + t91 + t90 - m1_11 * m2_8 + t151 - m2_11 * m1_8 + t93 - m1_10 * m2_7 - m1_12 * m2_9 - m2_10 * m1_7 + t154);
  double t303 = (t164 + t102 - m3_10 * m1_7 + t160 - m3_12 * m1_9 - m1_10 * m3_7 - m3_11 * m1_8 + t101 - m1_12 * m3_9 + t106 + t165 - m1_11 * m3_8);
  double t317 = (t125 + t119 - m3_12 * m2_9 - m3_10 * m2_7 - m2_11 * m3_8 - m3_11 * m2_8 + t122 + t179 - m2_12 * m3_9 + t181 - m2_10 * m3_7 + t185);

  m66dP(0,0) = t1 - 2 * m1_4 * m1_1 + t4 + t5 + t6 - 2 * m1_5 * m1_2 - 2 * m1_6 * m1_3 + t11 + t12;
  m66dP(0,1) = 2 * t26;
  m66dP(0,2) = 2 * t39;
  m66dP(0,3) = t40 + t41 + t42 + t43 + t44 + t45 - 2 * m2_5 * m2_2 - 2 * m2_6 * m2_3 - 2 * m2_4 * m2_1;
  m66dP(0,4) = 2 * t65;
  m66dP(0,5) = t66 - 2 * m3_4 * m3_1 + t69 + t70 + t71 + t72 - 2 * m3_6 * m3_3 + t75 - 2 * m3_5 * m3_2;
  m66dP(1,0) = -2 * m1_8 * m1_2 + t12 + t81 + t5 + t82 + t83 + t1 - 2 * m1_9 * m1_3 - 2 * m1_7 * m1_1;
  m66dP(1,1) = 2 * t98;
  m66dP(1,2) = 2 * t108;
  m66dP(1,3) = -2 * m2_8 * m2_2 - 2 * m2_9 * m2_3 + t113 - 2 * m2_7 * m2_1 + t116 + t117 + t41 + t43 + t45;
  m66dP(1,4) = 2 * t128;
  m66dP(1,5) = t75 + t70 + t129 + t71 + t130 - 2 * m3_9 * m3_3 + t133 - 2 * m3_8 * m3_2 - 2 * m3_7 * m3_1;
  m66dP(2,0) = -2 * m1_11 * m1_2 + t141 + t142 + t12 + t1 + t5 + t143 - 2 * m1_10 * m1_1 - 2 * m1_12 * m1_3;
  m66dP(2,1) = 2 * t158;
  m66dP(2,2) = 2 * t168;
  m66dP(2,3) = t169 + t41 + t43 + t45 + t170 + t171 - 2 * m2_12 * m2_3 - 2 * m2_10 * m2_1 - 2 * m2_11 * m2_2;
  m66dP(2,4) = 2 * t188;
  m66dP(2,5) = t71 - 2 * m3_12 * m3_3 + t75 + t191 + t70 + t192 + t193 - 2 * m3_10 * m3_1 - 2 * m3_11 * m3_2;
  m66dP(3,0) = -2 * m1_9 * m1_6 + t11 + t4 + t81 + t82 + t6 + t83 - 2 * m1_7 * m1_4 - 2 * m1_8 * m1_5;
  m66dP(3,1) = 2 * t212;
  m66dP(3,2) = 2 * t219;
  m66dP(3,3) = t117 + t113 - 2 * m2_8 * m2_5 - 2 * m2_7 * m2_4 - 2 * m2_9 * m2_6 + t116 + t40 + t42 + t44;
  m66dP(3,4) = 2 * t233;
  m66dP(3,5) = t129 - 2 * m3_9 * m3_6 + t69 + t66 + t72 + t133 - 2 * m3_8 * m3_5 - 2 * m3_7 * m3_4 + t130;
  m66dP(4,0) = t4 + t143 + t11 - 2 * m1_12 * m1_6 - 2 * m1_11 * m1_5 - 2 * m1_10 * m1_4 + t6 + t141 + t142;
  m66dP(4,1) = 2 * t254;
  m66dP(4,2) = 2 * t261;
  m66dP(4,3) = t170 + t171 + t169 - 2 * m2_10 * m2_4 - 2 * m2_11 * m2_5 - 2 * m2_12 * m2_6 + t40 + t42 + t44;
  m66dP(4,4) = 2 * t275;
  m66dP(4,5) = t69 + t66 + t72 - 2 * m3_12 * m3_6 - 2 * m3_10 * m3_4 + t193 - 2 * m3_11 * m3_5 + t192 + t191;
  m66dP(5,0) = t142 - 2 * m1_10 * m1_7 + t141 + t83 + t143 + t82 - 2 * m1_12 * m1_9 + t81 - 2 * m1_11 * m1_8;
  m66dP(5,1) = 2 * t296;
  m66dP(5,2) = 2 * t303;
  m66dP(5,3) = t171 + t170 - 2 * m2_12 * m2_9 - 2 * m2_10 * m2_7 - 2 * m2_11 * m2_8 + t113 + t169 + t116 + t117;
  m66dP(5,4) = 2 * t317;
  m66dP(5,5) = -2 * m3_11 * m3_8 + t193 + t133 + t129 + t192 + t130 - 2 * m3_12 * m3_9 + t191 - 2 * m3_10 * m3_7;

  return m66dP;
}

inline MatrixXd compute_constraint_distance_orthog_4param_9eq_10unk(const VectorXd vxdKm1, const VectorXd vxdKm2, const VectorXd vxdKm3, const VectorXd vxdKm4)
{
  MatrixXd m910dP(9,10);

  double m1_1 = vxdKm1(0);
  double m1_2 = vxdKm1(1);
  double m1_3 = vxdKm1(2);
  double m1_4 = vxdKm1(3);
  double m1_5 = vxdKm1(4);
  double m1_6 = vxdKm1(5);
  double m1_7 = vxdKm1(6);
  double m1_8 = vxdKm1(7);
  double m1_9 = vxdKm1(8);
  double m1_10 = vxdKm1(9);
  double m1_11 = vxdKm1(10);
  double m1_12 = vxdKm1(11);

  double m2_1 = vxdKm2(0);
  double m2_2 = vxdKm2(1);
  double m2_3 = vxdKm2(2);
  double m2_4 = vxdKm2(3);
  double m2_5 = vxdKm2(4);
  double m2_6 = vxdKm2(5);
  double m2_7 = vxdKm2(6);
  double m2_8 = vxdKm2(7);
  double m2_9 = vxdKm2(8);
  double m2_10 = vxdKm2(9);
  double m2_11 = vxdKm2(10);
  double m2_12 = vxdKm2(11);

  double m3_1 = vxdKm3(0);
  double m3_2 = vxdKm3(1);
  double m3_3 = vxdKm3(2);
  double m3_4 = vxdKm3(3);
  double m3_5 = vxdKm3(4);
  double m3_6 = vxdKm3(5);
  double m3_7 = vxdKm3(6);
  double m3_8 = vxdKm3(7);
  double m3_9 = vxdKm3(8);
  double m3_10 = vxdKm3(9);
  double m3_11 = vxdKm3(10);
  double m3_12 = vxdKm3(11);

  double m4_1 = vxdKm4(0);
  double m4_2 = vxdKm4(1);
  double m4_3 = vxdKm4(2);
  double m4_4 = vxdKm4(3);
  double m4_5 = vxdKm4(4);
  double m4_6 = vxdKm4(5);
  double m4_7 = vxdKm4(6);
  double m4_8 = vxdKm4(7);
  double m4_9 = vxdKm4(8);
  double m4_10 = vxdKm4(9);
  double m4_11 = vxdKm4(10);
  double m4_12 = vxdKm4(11);

  double t1 = (m1_1 * m1_1);
  double t2 = (m1_2 * m1_2);
  double t3 = (m1_6 * m1_6);
  double t4 = (m1_3 * m1_3);
  double t5 = (m1_5 * m1_5);
  double t6 = (m1_4 * m1_1);
  double t8 = (m1_5 * m1_2);
  double t10 = (m1_6 * m1_3);
  double t12 = (m1_4 * m1_4);
  double t14 = (m2_5 * m1_2);
  double t15 = m1_3 * m2_3;
  double t16 = (m2_6 * m1_3);
  double t17 = m1_6 * m2_6;
  double t18 = m1_1 * m2_1;
  double t19 = (m1_4 * m2_1);
  double t20 = m1_2 * m2_2;
  double t21 = (m1_5 * m2_2);
  double t22 = m1_5 * m2_5;
  double t23 = (m1_6 * m2_3);
  double t24 = m1_4 * m2_4;
  double t25 = (m2_4 * m1_1);
  double t26 = (-t14 + t15 - t16 + t17 + t18 - t19 + t20 - t21 + t22 - t23 + t24 - t25);
  double t27 = (m1_5 * m3_2);
  double t28 = m1_3 * m3_3;
  double t29 = (m1_6 * m3_3);
  double t30 = m1_4 * m3_4;
  double t31 = (m3_4 * m1_1);
  double t32 = m1_1 * m3_1;
  double t33 = m1_5 * m3_5;
  double t34 = m1_6 * m3_6;
  double t35 = m1_2 * m3_2;
  double t36 = (m1_4 * m3_1);
  double t37 = (m3_6 * m1_3);
  double t38 = (m3_5 * m1_2);
  double t39 = (-t27 + t28 - t29 + t30 - t31 + t32 + t33 + t34 + t35 - t36 - t37 - t38);
  double t40 = m1_3 * m4_3;
  double t41 = (m1_4 * m4_1);
  double t42 = m1_2 * m4_2;
  double t43 = m1_6 * m4_6;
  double t44 = (m1_5 * m4_2);
  double t45 = (m4_6 * m1_3);
  double t46 = m1_1 * m4_1;
  double t47 = m1_5 * m4_5;
  double t48 = m1_4 * m4_4;
  double t49 = (m4_5 * m1_2);
  double t50 = (m4_4 * m1_1);
  double t51 = (m1_6 * m4_3);
  double t52 = (t40 - t41 + t42 + t43 - t44 - t45 + t46 + t47 + t48 - t49 - t50 - t51);
  double t53 = (m2_5 * m2_2);
  double t55 = (m2_6 * m2_3);
  double t57 = (m2_4 * m2_4);
  double t58 = (m2_1 * m2_1);
  double t59 = (m2_5 * m2_5);
  double t60 = (m2_2 * m2_2);
  double t61 = (m2_6 * m2_6);
  double t62 = (m2_3 * m2_3);
  double t63 = (m2_4 * m2_1);
  double t66 = m2_4 * m3_4;
  double t67 = m2_5 * m3_5;
  double t68 = (m3_6 * m2_3);
  double t69 = (m2_5 * m3_2);
  double t70 = m2_3 * m3_3;
  double t71 = m2_2 * m3_2;
  double t72 = m2_6 * m3_6;
  double t73 = (m2_6 * m3_3);
  double t74 = (m3_5 * m2_2);
  double t75 = (m2_4 * m3_1);
  double t76 = (m3_4 * m2_1);
  double t77 = m2_1 * m3_1;
  double t78 = (t66 + t67 - t68 - t69 + t70 + t71 + t72 - t73 - t74 - t75 - t76 + t77);
  double t79 = (m2_6 * m4_3);
  double t80 = (m4_6 * m2_3);
  double t81 = m2_3 * m4_3;
  double t82 = m2_6 * m4_6;
  double t83 = (m2_4 * m4_1);
  double t84 = (m4_4 * m2_1);
  double t85 = m2_1 * m4_1;
  double t86 = m2_4 * m4_4;
  double t87 = (m2_5 * m4_2);
  double t88 = (m4_5 * m2_2);
  double t89 = m2_2 * m4_2;
  double t90 = m2_5 * m4_5;
  double t91 = (-t79 - t80 + t81 + t82 - t83 - t84 + t85 + t86 - t87 - t88 + t89 + t90);
  double t92 = (m3_4 * m3_1);
  double t94 = (m3_4 * m3_4);
  double t95 = (m3_1 * m3_1);
  double t96 = (m3_5 * m3_5);
  double t97 = (m3_2 * m3_2);
  double t98 = (m3_6 * m3_6);
  double t99 = (m3_3 * m3_3);
  double t100 = (m3_5 * m3_2);
  double t102 = (m3_6 * m3_3);
  double t105 = m3_6 * m4_6;
  double t106 = (m3_5 * m4_2);
  double t107 = m3_3 * m4_3;
  double t108 = (m4_5 * m3_2);
  double t109 = m3_2 * m4_2;
  double t110 = (m4_4 * m3_1);
  double t111 = (m4_6 * m3_3);
  double t112 = m3_4 * m4_4;
  double t113 = m3_5 * m4_5;
  double t114 = (m3_4 * m4_1);
  double t115 = (m3_6 * m4_3);
  double t116 = m3_1 * m4_1;
  double t117 = (t105 - t106 + t107 - t108 + t109 - t110 - t111 + t112 + t113 - t114 - t115 + t116);
  double t118 = (m4_3 * m4_3);
  double t119 = (m4_4 * m4_4);
  double t120 = (m4_2 * m4_2);
  double t121 = (m4_5 * m4_2);
  double t123 = (m4_6 * m4_3);
  double t125 = (m4_1 * m4_1);
  double t126 = (m4_6 * m4_6);
  double t127 = (m4_5 * m4_5);
  double t128 = (m4_4 * m4_1);
  double t131 = (m1_7 * m1_7);
  double t132 = (m1_8 * m1_8);
  double t133 = (m1_9 * m1_9);
  double t134 = (m1_8 * m1_2);
  double t136 = (m1_9 * m1_3);
  double t138 = (m1_7 * m1_1);
  double t141 = m1_9 * m2_9;
  double t142 = (m2_8 * m1_2);
  double t143 = (m2_7 * m1_1);
  double t144 = (m1_8 * m2_2);
  double t145 = (m2_9 * m1_3);
  double t146 = m1_8 * m2_8;
  double t147 = (m1_7 * m2_1);
  double t148 = (m1_9 * m2_3);
  double t149 = m1_7 * m2_7;
  double t150 = (t141 + t20 - t142 + t15 - t143 - t144 - t145 + t146 - t147 - t148 + t18 + t149);
  double t151 = (m1_9 * m3_3);
  double t152 = (m1_7 * m3_1);
  double t153 = m1_7 * m3_7;
  double t154 = (m3_9 * m1_3);
  double t155 = m1_9 * m3_9;
  double t156 = (m1_8 * m3_2);
  double t157 = (m3_8 * m1_2);
  double t158 = (m3_7 * m1_1);
  double t159 = m1_8 * m3_8;
  double t160 = (t28 + t32 - t151 - t152 + t153 - t154 + t155 - t156 - t157 - t158 + t35 + t159);
  double t161 = (m4_8 * m1_2);
  double t162 = (m1_8 * m4_2);
  double t163 = m1_9 * m4_9;
  double t164 = (m1_9 * m4_3);
  double t165 = m1_8 * m4_8;
  double t166 = (m4_7 * m1_1);
  double t167 = (m4_9 * m1_3);
  double t168 = m1_7 * m4_7;
  double t169 = (m1_7 * m4_1);
  double t170 = (-t161 - t162 + t42 + t163 - t164 + t165 - t166 + t46 - t167 + t40 + t168 - t169);
  double t171 = (m2_9 * m2_3);
  double t173 = (m2_8 * m2_2);
  double t175 = (m2_8 * m2_8);
  double t176 = (m2_9 * m2_9);
  double t177 = (m2_7 * m2_1);
  double t179 = (m2_7 * m2_7);
  double t181 = m2_8 * m3_8;
  double t182 = m2_7 * m3_7;
  double t183 = (m2_7 * m3_1);
  double t184 = (m3_8 * m2_2);
  double t185 = (m2_8 * m3_2);
  double t186 = (m3_7 * m2_1);
  double t187 = (m3_9 * m2_3);
  double t188 = m2_9 * m3_9;
  double t189 = (m2_9 * m3_3);
  double t190 = (t77 + t71 + t181 + t70 + t182 - t183 - t184 - t185 - t186 - t187 + t188 - t189);
  double t191 = (m2_8 * m4_2);
  double t192 = m2_7 * m4_7;
  double t193 = (m2_7 * m4_1);
  double t194 = (m4_7 * m2_1);
  double t195 = (m2_9 * m4_3);
  double t196 = m2_8 * m4_8;
  double t197 = m2_9 * m4_9;
  double t198 = (m4_8 * m2_2);
  double t199 = (m4_9 * m2_3);
  double t200 = (-t191 + t192 - t193 + t89 - t194 - t195 + t196 + t197 - t198 + t85 + t81 - t199);
  double t201 = (m3_9 * m3_3);
  double t203 = (m3_7 * m3_1);
  double t205 = (m3_8 * m3_2);
  double t207 = (m3_7 * m3_7);
  double t208 = (m3_8 * m3_8);
  double t209 = (m3_9 * m3_9);
  double t211 = (m3_7 * m4_1);
  double t212 = (m4_9 * m3_3);
  double t213 = (m4_8 * m3_2);
  double t214 = (m4_7 * m3_1);
  double t215 = (m3_8 * m4_2);
  double t216 = m3_9 * m4_9;
  double t217 = (m3_9 * m4_3);
  double t218 = m3_7 * m4_7;
  double t219 = m3_8 * m4_8;
  double t220 = (-t211 + t109 - t212 - t213 - t214 - t215 + t216 - t217 + t107 + t218 + t219 + t116);
  double t221 = (m4_9 * m4_9);
  double t222 = (m4_7 * m4_7);
  double t223 = (m4_9 * m4_3);
  double t225 = (m4_8 * m4_8);
  double t226 = (m4_8 * m4_2);
  double t228 = (m4_7 * m4_1);
  double t231 = (m1_12 * m1_12);
  double t232 = (m1_11 * m1_2);
  double t234 = (m1_10 * m1_10);
  double t235 = (m1_11 * m1_11);
  double t236 = (m1_10 * m1_1);
  double t238 = (m1_12 * m1_3);
  double t241 = (m1_12 * m2_12);
  double t242 = (m1_12 * m2_3);
  double t243 = (m2_12 * m1_3);
  double t244 = (m1_10 * m2_1);
  double t245 = (m2_11 * m1_2);
  double t246 = (m1_11 * m2_11);
  double t247 = (m1_11 * m2_2);
  double t248 = (m2_10 * m1_1);
  double t249 = (m1_10 * m2_10);
  double t250 = (t241 - t242 - t243 - t244 + t18 - t245 + t246 - t247 + t20 + t15 - t248 + t249);
  double t251 = (m1_12 * m3_3);
  double t252 = (m1_12 * m3_12);
  double t253 = (m3_12 * m1_3);
  double t254 = (m1_10 * m3_1);
  double t255 = (m3_10 * m1_1);
  double t256 = (m1_10 * m3_10);
  double t257 = (m1_11 * m3_2);
  double t258 = (m3_11 * m1_2);
  double t259 = (m1_11 * m3_11);
  double t260 = (-t251 + t28 + t252 - t253 - t254 - t255 + t35 + t256 - t257 + t32 - t258 + t259);
  double t261 = (m4_11 * m1_2);
  double t262 = (m1_10 * m4_1);
  double t263 = (m1_10 * m4_10);
  double t264 = (m1_12 * m4_12);
  double t265 = (m1_12 * m4_3);
  double t266 = (m4_12 * m1_3);
  double t267 = (m1_11 * m4_2);
  double t268 = (m1_11 * m4_11);
  double t269 = (m4_10 * m1_1);
  double t270 = (-t261 + t46 - t262 + t263 + t42 + t264 + t40 - t265 - t266 - t267 + t268 - t269);
  double t271 = (m2_11 * m2_2);
  double t273 = (m2_12 * m2_3);
  double t275 = (m2_11 * m2_11);
  double t276 = (m2_12 * m2_12);
  double t277 = (m2_10 * m2_10);
  double t278 = (m2_10 * m2_1);
  double t281 = (m3_10 * m2_1);
  double t282 = (m3_12 * m2_3);
  double t283 = (m2_11 * m3_2);
  double t284 = (m2_11 * m3_11);
  double t285 = (m2_10 * m3_10);
  double t286 = (m2_10 * m3_1);
  double t287 = (m2_12 * m3_3);
  double t288 = (m2_12 * m3_12);
  double t289 = (m3_11 * m2_2);
  double t290 = (-t281 - t282 + t70 - t283 + t284 + t285 - t286 - t287 + t288 + t71 + t77 - t289);
  double t291 = (m2_11 * m4_2);
  double t292 = (m4_11 * m2_2);
  double t293 = (m4_10 * m2_1);
  double t294 = (m2_12 * m4_12);
  double t295 = (m2_12 * m4_3);
  double t296 = (m2_10 * m4_1);
  double t297 = (m2_10 * m4_10);
  double t298 = (m2_11 * m4_11);
  double t299 = (m4_12 * m2_3);
  double t300 = (-t291 - t292 - t293 + t294 + t89 - t295 + t81 + t85 - t296 + t297 + t298 - t299);
  double t301 = (m3_10 * m3_1);
  double t303 = (m3_12 * m3_12);
  double t304 = (m3_12 * m3_3);
  double t306 = (m3_11 * m3_2);
  double t308 = (m3_10 * m3_10);
  double t309 = (m3_11 * m3_11);
  double t311 = (m3_12 * m4_3);
  double t312 = (m3_10 * m4_10);
  double t313 = (m3_12 * m4_12);
  double t314 = (m3_11 * m4_2);
  double t315 = (m4_12 * m3_3);
  double t316 = (m4_11 * m3_2);
  double t317 = (m3_11 * m4_11);
  double t318 = (m3_10 * m4_1);
  double t319 = (m4_10 * m3_1);
  double t320 = (-t311 + t107 + t312 + t313 - t314 - t315 + t109 - t316 + t317 - t318 - t319 + t116);
  double t321 = (m4_11 * m4_11);
  double t322 = (m4_10 * m4_10);
  double t323 = (m4_11 * m4_2);
  double t325 = (m4_12 * m4_3);
  double t327 = (m4_10 * m4_1);
  double t329 = (m4_12 * m4_12);
  double t331 = (m1_8 * m1_5);
  double t333 = (m1_9 * m1_6);
  double t335 = (m1_7 * m1_4);
  double t338 = (m2_9 * m1_6);
  double t339 = (m1_7 * m2_4);
  double t340 = (m1_9 * m2_6);
  double t341 = (m2_7 * m1_4);
  double t342 = (m2_8 * m1_5);
  double t343 = (m1_8 * m2_5);
  double t344 = (t22 - t338 - t339 - t340 + t17 + t146 - t341 - t342 + t141 + t24 + t149 - t343);
  double t345 = (m3_9 * m1_6);
  double t346 = (m1_7 * m3_4);
  double t347 = (m1_9 * m3_6);
  double t348 = (m3_7 * m1_4);
  double t349 = (m3_8 * m1_5);
  double t350 = (m1_8 * m3_5);
  double t351 = (t153 - t345 - t346 + t155 + t30 - t347 - t348 + t33 - t349 + t159 + t34 - t350);
  double t352 = (m1_8 * m4_5);
  double t353 = (m4_8 * m1_5);
  double t354 = (m4_7 * m1_4);
  double t355 = (m1_9 * m4_6);
  double t356 = (m4_9 * m1_6);
  double t357 = (m1_7 * m4_4);
  double t358 = (-t352 + t48 + t165 - t353 - t354 + t163 + t43 + t168 + t47 - t355 - t356 - t357);
  double t359 = (m2_7 * m2_4);
  double t361 = (m2_9 * m2_6);
  double t363 = (m2_8 * m2_5);
  double t366 = (m2_7 * m3_4);
  double t367 = (m2_8 * m3_5);
  double t368 = (m3_7 * m2_4);
  double t369 = (m3_9 * m2_6);
  double t370 = (m2_9 * m3_6);
  double t371 = (m3_8 * m2_5);
  double t372 = (-t366 - t367 + t182 + t66 + t188 - t368 - t369 + t67 - t370 + t72 - t371 + t181);
  double t373 = (m2_8 * m4_5);
  double t374 = (m4_7 * m2_4);
  double t375 = (m2_7 * m4_4);
  double t376 = (m4_8 * m2_5);
  double t377 = (m2_9 * m4_6);
  double t378 = (m4_9 * m2_6);
  double t379 = (t196 + t82 - t373 + t90 - t374 + t86 + t192 + t197 - t375 - t376 - t377 - t378);
  double t380 = (m3_7 * m3_4);
  double t382 = (m3_8 * m3_5);
  double t384 = (m3_9 * m3_6);
  double t387 = (m4_9 * m3_6);
  double t388 = (m4_7 * m3_4);
  double t389 = (m3_9 * m4_6);
  double t390 = (m4_8 * m3_5);
  double t391 = (m3_8 * m4_5);
  double t392 = (m3_7 * m4_4);
  double t393 = (-t387 - t388 - t389 + t105 + t218 - t390 + t112 + t216 + t219 - t391 - t392 + t113);
  double t394 = (m4_9 * m4_6);
  double t396 = (m4_8 * m4_5);
  double t398 = (m4_7 * m4_4);
  double t401 = (m1_12 * m1_6);
  double t403 = (m1_10 * m1_4);
  double t405 = (m1_11 * m1_5);
  double t408 = (m2_11 * m1_5);
  double t409 = (m2_12 * m1_6);
  double t410 = (m1_10 * m2_4);
  double t411 = (m1_11 * m2_5);
  double t412 = (m2_10 * m1_4);
  double t413 = (m1_12 * m2_6);
  double t414 = (t246 + t22 + t241 - t408 + t249 - t409 + t24 - t410 - t411 - t412 + t17 - t413);
  double t415 = (m1_10 * m3_4);
  double t416 = (m3_12 * m1_6);
  double t417 = (m3_10 * m1_4);
  double t418 = (m1_12 * m3_6);
  double t419 = (m3_11 * m1_5);
  double t420 = (m1_11 * m3_5);
  double t421 = (-t415 + t33 + t252 - t416 - t417 - t418 + t30 + t34 + t259 + t256 - t419 - t420);
  double t422 = (m1_12 * m4_6);
  double t423 = (m1_11 * m4_5);
  double t424 = (m4_11 * m1_5);
  double t425 = (m4_10 * m1_4);
  double t426 = (m4_12 * m1_6);
  double t427 = (m1_10 * m4_4);
  double t428 = (t263 - t422 + t43 - t423 + t48 - t424 - t425 + t264 - t426 - t427 + t47 + t268);
  double t429 = (m2_12 * m2_6);
  double t431 = (m2_11 * m2_5);
  double t433 = (m2_10 * m2_4);
  double t436 = (m2_10 * m3_4);
  double t437 = (m3_11 * m2_5);
  double t438 = (m2_12 * m3_6);
  double t439 = (m3_10 * m2_4);
  double t440 = (m3_12 * m2_6);
  double t441 = (m2_11 * m3_5);
  double t442 = (t288 + t72 - t436 - t437 - t438 + t285 + t67 + t66 - t439 - t440 + t284 - t441);
  double t443 = (m4_11 * m2_5);
  double t444 = (m2_12 * m4_6);
  double t445 = (m4_12 * m2_6);
  double t446 = (m2_11 * m4_5);
  double t447 = (m2_10 * m4_4);
  double t448 = (m4_10 * m2_4);
  double t449 = (t298 + t294 + t297 + t90 - t443 - t444 + t86 - t445 - t446 - t447 + t82 - t448);
  double t450 = (m3_10 * m3_4);
  double t452 = (m3_12 * m3_6);
  double t454 = (m3_11 * m3_5);
  double t457 = (m3_12 * m4_6);
  double t458 = (m3_11 * m4_5);
  double t459 = (m4_12 * m3_6);
  double t460 = (m3_10 * m4_4);
  double t461 = (m4_10 * m3_4);
  double t462 = (m4_11 * m3_5);
  double t463 = (-t457 - t458 + t313 - t459 + t105 + t312 - t460 + t317 - t461 + t113 - t462 + t112);
  double t464 = (m4_12 * m4_6);
  double t466 = (m4_10 * m4_4);
  double t468 = (m4_11 * m4_5);
  double t471 = (m1_12 * m1_9);
  double t473 = (m1_10 * m1_7);
  double t475 = (m1_11 * m1_8);
  double t478 = (m2_12 * m1_9);
  double t479 = (m2_11 * m1_8);
  double t480 = (m1_10 * m2_7);
  double t481 = (m1_12 * m2_9);
  double t482 = (m2_10 * m1_7);
  double t483 = (m1_11 * m2_8);
  double t484 = (t149 - t478 - t479 - t480 - t481 - t482 + t246 + t249 + t241 + t141 + t146 - t483);
  double t485 = (m1_11 * m3_8);
  double t486 = (m3_12 * m1_9);
  double t487 = (m3_11 * m1_8);
  double t488 = (m1_12 * m3_9);
  double t489 = (m1_10 * m3_7);
  double t490 = (m3_10 * m1_7);
  double t491 = (t159 + t256 - t485 + t252 - t486 - t487 - t488 + t155 - t489 - t490 + t259 + t153);
  double t492 = (m1_11 * m4_8);
  double t493 = (m4_10 * m1_7);
  double t494 = (m4_11 * m1_8);
  double t495 = (m1_10 * m4_7);
  double t496 = (m4_12 * m1_9);
  double t497 = (m1_12 * m4_9);
  double t498 = (-t492 + t165 + t163 - t493 - t494 - t495 + t168 + t263 + t264 + t268 - t496 - t497);
  double t499 = (m2_12 * m2_9);
  double t501 = (m2_11 * m2_8);
  double t503 = (m2_10 * m2_7);
  double t506 = (m2_10 * m3_7);
  double t507 = (m3_10 * m2_7);
  double t508 = (m3_12 * m2_9);
  double t509 = (m3_11 * m2_8);
  double t510 = (m2_11 * m3_8);
  double t511 = (m2_12 * m3_9);
  double t512 = (t288 - t506 - t507 - t508 + t285 + t182 - t509 - t510 + t181 - t511 + t188 + t284);
  double t513 = (m2_11 * m4_8);
  double t514 = (m4_11 * m2_8);
  double t515 = (m2_12 * m4_9);
  double t516 = (m4_10 * m2_7);
  double t517 = (m4_12 * m2_9);
  double t518 = (m2_10 * m4_7);
  double t519 = (-t513 - t514 - t515 - t516 + t196 + t192 + t297 + t294 - t517 - t518 + t197 + t298);
  double t520 = (m3_10 * m3_7);
  double t522 = (m3_11 * m3_8);
  double t524 = (m3_12 * m3_9);
  double t527 = (m3_12 * m4_9);
  double t528 = (m4_12 * m3_9);
  double t529 = (m3_11 * m4_8);
  double t530 = (m4_10 * m3_7);
  double t531 = (m4_11 * m3_8);
  double t532 = (m3_10 * m4_7);
  double t533 = (-t527 - t528 + t313 + t218 - t529 + t216 + t317 + t312 - t530 + t219 - t531 - t532);
  double t534 = (m4_11 * m4_8);
  double t536 = (m4_12 * m4_9);
  double t538 = (m4_10 * m4_7);
  double t541 = (-t232 - t236 - t401 + t6 + t8 + t10 - t405 - t403 + t234 + t235 - t238 + t231);
  double t543 = 2 * t241;
  double t544 = 2 * t249;
  double t545 = 2 * t246;
  double t546 = t543 - t408 - t242 + t544 - t244 + t16 - t411 + t25 + t545 - t247 + t19;
  double t548 = 2 * t252;
  double t550 = 2 * t259;
  double t551 = 2 * t256;
  double t552 = -t420 - t419 + t550 + t29 + t36 - t251 - t255 + t38 + t31 + t551 - t415;
  double t555 = 2 * t268;
  double t556 = 2 * t263;
  double t557 = 2 * t264;
  double t558 = t41 + t51 + t555 - t261 - t423 - t265 + t556 - t426 - t427 - t269 + t557;
  double t560 = (-t278 - t433 - t429 + t63 + t53 + t55 + t275 + t276 - t273 - t271 + t277 - t431);
  double t561 = 2 * t288;
  double t562 = 2 * t284;
  double t564 = 2 * t285;
  double t565 = t75 + t76 - t436 - t281 - t440 + t74 - t289 - t441 + t68 - t283 + t564;
  double t567 = 2 * t297;
  double t568 = 2 * t298;
  double t570 = 2 * t294;
  double t571 = t79 - t296 - t448 - t446 - t293 + t80 + t84 + t570 - t291 + t83 - t445;
  double t573 = (t303 - t304 - t301 - t450 - t452 - t306 - t454 + t308 + t309 + t92 + t100 + t102);
  double t574 = 2 * t312;
  double t575 = 2 * t317;
  double t577 = 2 * t313;
  double t578 = -t311 + t577 - t461 - t462 + t111 + t114 + t115 - t318 + t106 - t319 - t457;
  double t580 = (t329 + t121 + t128 - t325 - t327 - t323 - t468 + t123 + t321 - t466 - t464 + t322);
  double t581 = (-t475 - t471 - t232 + t234 + t235 + t134 + t136 + t138 - t236 - t238 - t473 + t231);
  double t583 = -t248 + t143 - t479 - t483 + t144 - t482 + t543 - t242 + t545 - t247 - t244;
  double t586 = t548 + t152 + t151 - t488 - t489 + t551 + t158 + t156 - t490 - t253 + t154;
  double t589 = -t497 + t164 + t167 + t169 + t166 + t557 + t556 + t161 + t555 - t496 - t269;
  double t591 = (t276 + t277 + t177 - t503 + t171 - t499 - t278 + t173 + t275 - t273 - t271 - t501);
  double t593 = -t282 + t184 + t185 - t509 - t506 - t281 - t510 + t189 - t511 + t186 - t289;
  double t596 = -t515 + t567 - t518 + t568 - t513 + t194 - t293 + t191 - t517 - t292 + t199;
  double t598 = (-t306 + t303 - t301 - t522 - t524 - t520 + t203 - t304 + t201 + t308 + t309 + t205);
  double t600 = -t319 + t213 + t574 + t215 + t212 - t314 + t575 + t217 - t532 - t530 - t531;
  double t602 = (-t327 + t321 + t329 + t226 + t322 - t325 - t536 + t228 - t538 + t223 - t534 - t323);
  double t603 = (-t475 - t473 - t405 + t231 + t235 + t335 - t401 - t471 + t234 - t403 + t331 + t333);
  double t605 = t339 - t413 - t479 - t409 + t544 - t411 - t412 + t342 - t483 - t410 + t341;
  double t608 = -t417 + t551 - t489 + t348 + t350 - t486 + t349 + t550 + t347 - t416 - t415;
  double t611 = t556 + t557 - t497 - t492 - t427 - t496 + t555 - t426 + t355 + t356 + t357;
  double t613 = (t359 - t429 - t499 + t276 - t433 - t501 + t363 + t275 - t503 + t361 - t431 + t277);
  double t615 = t369 - t507 - t509 - t440 - t506 + t371 - t510 + t562 - t511 - t439 - t441;
  double t618 = -t444 - t514 + t378 + t570 + t377 - t447 + t374 + t376 - t517 + t373 - t515;
  double t620 = (-t522 + t384 - t524 + t303 - t450 - t452 - t520 - t454 + t308 + t309 + t380 + t382);
  double t622 = t575 + t389 + t390 - t461 - t462 - t529 - t532 - t530 + t391 - t531 + t392;
  double t624 = (t396 - t466 - t534 + t394 + t321 + t329 - t464 + t322 - t536 - t538 + t398 - t468);

  m910dP(0,0) = t1 + t2 + t3 + t4 + t5 - 2 * t6 - 2 * t8 - 2 * t10 + t12;
  m910dP(0,1) = 2 * t26;
  m910dP(0,2) = 2 * t39;
  m910dP(0,3) = 2 * t52;
  m910dP(0,4) = -2 * t53 - 2 * t55 + t57 + t58 + t59 + t60 + t61 + t62 - 2 * t63;
  m910dP(0,5) = 2 * t78;
  m910dP(0,6) = 2 * t91;
  m910dP(0,7) = -2 * t92 + t94 + t95 + t96 + t97 + t98 + t99 - 2 * t100 - 2 * t102;
  m910dP(0,8) = 2 * t117;
  m910dP(0,9) = t118 + t119 + t120 - 2 * t121 - 2 * t123 + t125 + t126 + t127 - 2 * t128;
  m910dP(1,0) = t1 + t4 + t131 + t132 + t133 - 2 * t134 - 2 * t136 - 2 * t138 + t2;
  m910dP(1,1) = 2 * t150;
  m910dP(1,2) = 2 * t160;
  m910dP(1,3) = 2 * t170;
  m910dP(1,4) = -2 * t171 - 2 * t173 + t60 + t62 + t58 + t175 + t176 - 2 * t177 + t179;
  m910dP(1,5) = 2 * t190;
  m910dP(1,6) = 2 * t200;
  m910dP(1,7) = -2 * t201 - 2 * t203 - 2 * t205 + t207 + t208 + t209 + t95 + t97 + t99;
  m910dP(1,8) = 2 * t220;
  m910dP(1,9) = t221 + t120 + t222 - 2 * t223 + t225 + t118 - 2 * t226 + t125 - 2 * t228;
  m910dP(2,0) = t231 - 2 * t232 + t2 + t234 + t235 + t1 - 2 * t236 - 2 * t238 + t4;
  m910dP(2,1) = 2 * t250;
  m910dP(2,2) = 2 * t260;
  m910dP(2,3) = 2 * t270;
  m910dP(2,4) = -2 * t271 - 2 * t273 + t275 + t276 + t277 + t58 + t60 + t62 - 2 * t278;
  m910dP(2,5) = 2 * t290;
  m910dP(2,6) = 2 * t300;
  m910dP(2,7) = -2 * t301 + t303 - 2 * t304 - 2 * t306 + t308 + t309 + t95 + t97 + t99;
  m910dP(2,8) = 2 * t320;
  m910dP(2,9) = t118 + t321 + t322 - 2 * t323 - 2 * t325 - 2 * t327 + t120 + t329 + t125;
  m910dP(3,0) = t3 - 2 * t331 - 2 * t333 - 2 * t335 + t132 + t133 + t131 + t12 + t5;
  m910dP(3,1) = 2 * t344;
  m910dP(3,2) = 2 * t351;
  m910dP(3,3) = 2 * t358;
  m910dP(3,4) = -2 * t359 + t59 - 2 * t361 - 2 * t363 + t57 + t176 + t61 + t179 + t175;
  m910dP(3,5) = 2 * t372;
  m910dP(3,6) = 2 * t379;
  m910dP(3,7) = t209 - 2 * t380 - 2 * t382 - 2 * t384 + t208 + t94 + t96 + t98 + t207;
  m910dP(3,8) = 2 * t393;
  m910dP(3,9) = t126 + t225 + t127 - 2 * t394 - 2 * t396 - 2 * t398 + t119 + t222 + t221;
  m910dP(4,0) = -2 * t401 + t12 + t3 - 2 * t403 + t231 + t5 - 2 * t405 + t234 + t235;
  m910dP(4,1) = 2 * t414;
  m910dP(4,2) = 2 * t421;
  m910dP(4,3) = 2 * t428;
  m910dP(4,4) = -2 * t429 + t276 + t57 + t277 + t61 + t275 + t59 - 2 * t431 - 2 * t433;
  m910dP(4,5) = 2 * t442;
  m910dP(4,6) = 2 * t449;
  m910dP(4,7) = t303 - 2 * t450 - 2 * t452 - 2 * t454 + t308 + t309 + t94 + t96 + t98;
  m910dP(4,8) = 2 * t463;
  m910dP(4,9) = t126 + t119 + t321 + t329 - 2 * t464 - 2 * t466 - 2 * t468 + t322 + t127;
  m910dP(5,0) = t132 + t133 + t131 - 2 * t471 + t231 - 2 * t473 - 2 * t475 + t234 + t235;
  m910dP(5,1) = 2 * t484;
  m910dP(5,2) = 2 * t491;
  m910dP(5,3) = 2 * t498;
  m910dP(5,4) = -2 * t499 + t175 + t275 + t179 - 2 * t501 + t276 + t176 - 2 * t503 + t277;
  m910dP(5,5) = 2 * t512;
  m910dP(5,6) = 2 * t519;
  m910dP(5,7) = t303 + t207 - 2 * t520 - 2 * t522 - 2 * t524 + t308 + t309 + t208 + t209;
  m910dP(5,8) = 2 * t533;
  m910dP(5,9) = t329 - 2 * t534 + t321 + t222 + t221 + t225 + t322 - 2 * t536 - 2 * t538;
  m910dP(6,0) = t541;
  m910dP(6,1) = -t248 + t21 - t410 + t14 - t243 - t412 - t245 - t409 - t413 + t23 + t546;
  m910dP(6,2) = -t417 - t258 - t253 - t257 - t254 + t548 - t418 - t416 + t37 + t27 + t552;
  m910dP(6,3) = -t425 - t422 + t44 + t50 + t45 - t424 - t266 - t267 - t262 + t49 + t558;
  m910dP(6,4) = t560;
  m910dP(6,5) = -t286 - t287 - t437 + t73 - t439 + t561 + t562 - t282 - t438 + t69 + t565;
  m910dP(6,6) = -t295 + t567 - t447 + t568 + t87 + t88 - t443 - t444 - t299 - t292 + t571;
  m910dP(6,7) = t573;
  m910dP(6,8) = -t459 - t316 - t315 - t314 - t458 + t574 + t108 - t460 + t110 + t575 + t578;
  m910dP(6,9) = t580;
  m910dP(7,0) = t581;
  m910dP(7,1) = -t478 - t481 - t480 - t243 + t148 + t145 + t544 + t147 + t142 - t245 + t583;
  m910dP(7,2) = t157 - t485 - t251 + t550 - t486 - t487 - t258 - t257 - t255 - t254 + t586;
  m910dP(7,3) = -t261 - t495 - t262 + t162 - t492 - t265 - t266 - t494 - t267 - t493 + t589;
  m910dP(7,4) = t591;
  m910dP(7,5) = t561 - t286 + t187 + t564 - t283 - t507 + t562 - t508 - t287 + t183 + t593;
  m910dP(7,6) = -t516 - t296 + t198 - t291 + t570 - t295 + t195 - t299 + t193 - t514 + t596;
  m910dP(7,7) = t598;
  m910dP(7,8) = -t528 - t316 - t527 - t318 + t577 - t315 + t214 - t529 - t311 + t211 + t600;
  m910dP(7,9) = t602;
  m910dP(8,0) = t603;
  m910dP(8,1) = t338 - t408 - t480 - t482 + t340 - t478 + t545 + t343 + t543 - t481 + t605;
  m910dP(8,2) = -t418 + t346 - t419 + t548 + t345 - t488 - t420 - t487 - t485 - t490 + t608;
  m910dP(8,3) = -t425 + t354 - t493 - t424 - t422 + t352 - t423 - t494 + t353 - t495 + t611;
  m910dP(8,4) = t613;
  m910dP(8,5) = t368 + t366 + t564 + t370 + t561 - t508 - t436 - t438 - t437 + t367 + t615;
  m910dP(8,6) = -t516 + t375 - t445 - t448 + t568 - t443 - t513 - t518 + t567 - t446 + t618;
  m910dP(8,7) = t620;
  m910dP(8,8) = t577 + t387 - t458 - t459 - t460 + t574 - t528 - t527 + t388 - t457 + t622;
  m910dP(8,9) = t624;

  return m910dP;
}

inline double reProjErr(const MatrixXd mxdObjPts, const MatrixXd mxdImgPts, const Matrix3d m3dR, const Vector3d v3dt, const Matrix3d m3dK)
{
  MatrixXd mxdPose(3,4);
  mxdPose.block(0,0,3,3) = m3dR;
  mxdPose.block(0,3,3,1) = v3dt;

  MatrixXd mxdReProj = m3dK * mxdPose * mxdObjPts;

  for (unsigned i = 0; i < mxdReProj.cols() ; i++)
  {
    mxdReProj(0,i) = mxdReProj(0,i)/mxdReProj(2,i);
    mxdReProj(1,i) = mxdReProj(1,i)/mxdReProj(2,i);
    mxdReProj(2,i) = mxdReProj(2,i)/mxdReProj(2,i);
  }

  return (mxdImgPts - mxdReProj).norm();
}

inline MatrixXd compute_permutation_constraint(MatrixXd mxdKd)
{
  unsigned uinDims = mxdKd.cols();
  unsigned uin = 4;

  Matrix4d mxdidx = Matrix4d::Zero();
  mxdidx << 0, 1, 2, 3,
       1, 4, 5, 6,
       2, 5, 7, 8,
       3, 6, 8, 9;

  unsigned uinRowsK = 30;
  unsigned uinColsK = uinDims*(uinDims+1)/2;

  MatrixXd mxdK = MatrixXd::Zero(uinRowsK, uinColsK);

  unsigned t = 0;
  for (unsigned i=0; i<uin; i++)
  {
    for (unsigned j=i+1; j<uin; j++)
    {
      unsigned offset = 0;
      for (unsigned a=0; a<uinDims; a++)
      {
        for (unsigned b=a; b<uinDims; b++)
        {
          if (a == b)
          {
            mxdK(t,offset) = mxdKd((unsigned)mxdidx(i,i),a)*mxdKd((unsigned)mxdidx(j,j),a) -
                mxdKd((unsigned)mxdidx(i,j),a)*mxdKd((unsigned)mxdidx(i,j),a);
          }

          else
          {
            mxdK(t,offset) = mxdKd((unsigned)mxdidx(i,i),a)*mxdKd((unsigned)mxdidx(j,j),b) -
                mxdKd((unsigned)mxdidx(i,j),a)*mxdKd((unsigned)mxdidx(i,j),b) +
                mxdKd((unsigned)mxdidx(i,i),b)*mxdKd((unsigned)mxdidx(j,j),a) -
                mxdKd((unsigned)mxdidx(i,j),b)*mxdKd((unsigned)mxdidx(i,j),a);
          }
          offset = offset + 1;
        }
      }
      t = t + 1;
    }
  }

  for (unsigned k=0; k<uin; k++)
  {
    for (unsigned j=k; j<uin; j++)
    {
      for (unsigned i=0; i<uin; i++)
      {
        if (i!=j && i!=k)
        {
          unsigned offset = 1;
          for (unsigned a=0; a<uinDims; a++)
          {
            for (unsigned b=a; b<uinDims; b++)
            {
              if (a == b)
              {
                mxdK(t,offset) = mxdKd((unsigned)mxdidx(i,j),a)*mxdKd((unsigned)mxdidx(i,k),a) -
                    mxdKd((unsigned)mxdidx(i,i),a)*mxdKd((unsigned)mxdidx(j,k),a);
              }

              else
              {
                mxdK(t,offset) = mxdKd((unsigned)mxdidx(i,j),a)*mxdKd((unsigned)mxdidx(i,k),b) -
                    mxdKd((unsigned)mxdidx(i,i),a)*mxdKd((unsigned)mxdidx(j,k),b) +
                    mxdKd((unsigned)mxdidx(i,j),b)*mxdKd((unsigned)mxdidx(i,k),a) -
                    mxdKd((unsigned)mxdidx(i,i),b)*mxdKd((unsigned)mxdidx(j,k),a);
              }

              offset=offset + 1;
            }
          }
          t = t + 1;
        }
      }
    }
  }

  return mxdK;
}

inline double sgn(double val)
{
  if (val < 0)
    return -1;
  else
    return 1;
}

pose EPNP(object r3ObjectPoints, image r2ImagePoints, camera m3Camera)
{
  pose se3Pose;

  if (r3ObjectPoints.getnCols() != r2ImagePoints.getnCols())
  {
    cout << "Error EPNP(): Point correspondence mismatch" << endl;
  }

  MatrixXd mxdObjectPoints = r3ObjectPoints.getObjectPoints();
  MatrixXd mxdImagePoints = r2ImagePoints.getImagePoints();
  Matrix3d m3dK = m3Camera.getIntrinsic();

  unsigned uinPts = mxdObjectPoints.cols();

  MatrixXd mxdObject = mxdObjectPoints.block(0,0,3,uinPts);
  MatrixXd mxdImage = mxdImagePoints.block(0,0,2,uinPts);

  double dthreshold = 20;

  Matrix4d m44dC = Matrix4d::Identity();
  m44dC(3,0) = 1;
  m44dC(3,1) = 1;
  m44dC(3,2) = 1;

  MatrixXd m4ndAlphas = m44dC.inverse() * mxdObjectPoints;
  MatrixXd mn4dAlphas = m4ndAlphas.transpose();
  MatrixXd mxdM = computeM(mxdImage, mn4dAlphas, m3dK);

  MatrixXd mxdMtM = mxdM.transpose() * mxdM;

  EigenSolver<MatrixXd> eigMtM;
  eigMtM.compute(mxdMtM);

  MatrixXd mxdV = eigMtM.eigenvectors().real();
  VectorXd vxdS = eigMtM.eigenvalues().real();
  MatrixXd mxdKer = MatrixXd::Zero(12,4);

  VectorXd::Index minIdx;
  double dmaxSval = vxdS.maxCoeff();

  vxdS.minCoeff(&minIdx);
  mxdKer.col(3) = -1*mxdV.col(minIdx);
  vxdS(minIdx) = dmaxSval;

  vxdS.minCoeff(&minIdx);
  mxdKer.col(2) = -1*mxdV.col(minIdx);
  vxdS(minIdx) = dmaxSval;

  vxdS.minCoeff(&minIdx);
  mxdKer.col(1) = mxdV.col(minIdx);
  vxdS(minIdx) = dmaxSval;

  vxdS.minCoeff(&minIdx);
  mxdKer.col(0) = mxdV.col(minIdx);

  MatrixXd mxdXc = computeCameraCoords(mxdKer.col(3), MatrixXd::Identity(4,3), mn4dAlphas, mxdObject);

  /* Dimension = 1 */
  pose se3Pose1 = absoluteOrientation(mxdXc.transpose(),mxdObject);
  double derror1 = reProjErr(mxdObjectPoints, mxdImagePoints, se3Pose1.getRotationMatrix(), se3Pose1.getTranstalion(), m3dK);

  /* Dimension = 2 */
  MatrixXd m63dP = compute_constraint_distance_2param_6eq_3unk(mxdKer.col(2),mxdKer.col(3));
  VectorXd vxdDsq(6);
  vxdDsq << 2e00, 2e00, 1e00, 2e00, 1e00, 1e00;

  Vector3d v3dBetas = (m63dP.transpose() * m63dP).inverse() * m63dP.transpose() * vxdDsq;
  double dBeta1 = sqrt(abs(v3dBetas(0)));
  double dBeta2 = sqrt(abs(v3dBetas(2))) * sgn(v3dBetas(1))*sgn(v3dBetas(0));

  VectorXd vxdX2 = dBeta1*mxdKer.col(2) + dBeta2*mxdKer.col(3);
  MatrixXd mxdXc2 = computeCameraCoords(vxdX2, MatrixXd::Identity(4,3), mn4dAlphas, mxdObject);

  pose se3Pose2 = absoluteOrientation(mxdXc2.transpose(),mxdObject);
  double derror2 = reProjErr(mxdObjectPoints, mxdImagePoints, se3Pose2.getRotationMatrix(), se3Pose2.getTranstalion(), m3dK);

  if (derror1 > dthreshold && derror2 > dthreshold)
  {
    /* Dimension = 3 */
    MatrixXd m66dP = compute_constraint_distance_3param_6eq_6unk(mxdKer.col(1), mxdKer.col(2),mxdKer.col(3));
    VectorXd x6dBetas = m66dP.inverse() * vxdDsq;
    dBeta1 = sqrt(abs(x6dBetas(0)));
    dBeta2 = sqrt(abs(x6dBetas(3))) * sgn(x6dBetas(1)) * sgn(x6dBetas(0));
    double dBeta3 = sqrt(abs(x6dBetas(5))) * sgn(x6dBetas(2)) * sgn(x6dBetas(0));

    VectorXd vxdX3 = dBeta1*mxdKer.col(1) + dBeta2*mxdKer.col(2) + dBeta3*mxdKer.col(3);
    MatrixXd mxdXc3 = computeCameraCoords(vxdX3, MatrixXd::Identity(4,3), mn4dAlphas, mxdObject);

    pose se3Pose3 = absoluteOrientation(mxdXc3.transpose(),mxdObject);
    double derror3 = reProjErr(mxdObjectPoints, mxdImagePoints, se3Pose3.getRotationMatrix(), se3Pose3.getTranstalion(), m3dK);

    if (derror1 < derror2 && derror1 < derror3)
      se3Pose = se3Pose1;

    else if (derror2 < derror1 && derror2 < derror3)
      se3Pose = se3Pose2;

    else
      se3Pose = se3Pose3;
  }

  else
  {
    if (derror1 < derror2)
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
