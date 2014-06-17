/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#ifndef POSE_HPP__
#define POSE_HPP__

#include <iostream>
#include "3rdParty/Eigen/Eigen"
#include "Camera.hpp"
#include "Object.hpp"
#include "Image.hpp"

using namespace Eigen;

class rotation {

protected:
  Matrix3d m3dRotation;

public:
  /* Constructors */
  rotation();
  rotation(Matrix3d m3dRotation);
  rotation(Vector3d v3dRotation);
  rotation(double dax, double day, double daz);

  /* Get Functions */
  Matrix3d getRotationMatrix(void);
  Vector3d getRotationEuler(void);

  /* Set Functions */
  void setRotation(Matrix3d m3dRotation);
  void setRotation(Vector3d v3dRotation);
  void setRotation(double dax, double day, double daz);
};

class pose {

protected:
  MatrixXd mxdPose;
  rotation m3dRotation;
  Vector3d v3dTranslation;
  unsigned uiIterations;
  double   dreprojError;
  double   dtime;

public:
  /* Constructors */
  pose();
  pose(MatrixXd mxdPose);
  pose(Matrix3d m3dRotation, Vector3d v3dTranslation);
  pose(Vector3d v3dEulerAngles, Vector3d v3dTranslation);
  pose(VectorXd v6dPose);
  pose(double dax, double day, double daz, Vector3d v3dTranslation);
  pose(double dax, double day, double daz, double dtx, double dty, double dtz);

  /* Get Functions */
  MatrixXd getPoseMatrix(void);
  VectorXd getPoseVector(void);
  Matrix3d getRotationMatrix(void);
  Vector3d getRotationEuler(void);
  Vector4d getRotationQuaternion(void);
  Vector3d getTranstalion(void);
  unsigned getIterations(void);
  double   getReprojError(void);
  double   getTime(void);

  /* Set Functions */
  void setPose(MatrixXd mxdPose);
  void setPose(Matrix3d m3dRotation, Vector3d v3dTranslation);
  void setIterations(unsigned uinIterations);
  void setReprojError(double drprojError);
  void setTime(double deTime);

  /* Functions */
  void generatePose(double ax, double ay, double az,\
      double tx, double ty, double tz,\
      double dax, double day, double daz,\
      double dtx, double dty, double dtz);
};

/* Functions */
/* DLT */
pose DLT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera);
pose normalizedDLT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera);
pose coplanarDLT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera);

/* ePnP */
pose EPNP(object r3ObjectPoints, image r2ImagePoints, camera m3Camera);

/* POSIT */
pose POSIT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, unsigned uiMaxIterations);
pose coplanarPOSIT(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, unsigned uiMaxIterations);

/* LM */
pose LM(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, pose pInitPose, unsigned uiMaxIterations);
pose coplanarLM(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, pose pInitPose, unsigned uiMaxIterations);

/* OI */
pose OI(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, pose pInitPose, unsigned uiMaxIterations);
pose coplanarOI(object r3ObjectPoints, image r2ImagePoints, camera m3Camera, pose pInitPose, unsigned uiMaxIterations);

/* Helper Functions */
double quaternionError(Vector4d v4dQ1, Vector4d v4dQ2);
double translationError(Vector3d v3dTr1, Vector3d v3dTr2);
double rotationError(Matrix3d m3dR1, Matrix3d m3dR2);

#endif /* POSE_HPP_ */
