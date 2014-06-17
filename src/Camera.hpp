/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#ifndef POSE_HPP_
#define POSE_HPP_

#include "3rdParty/Eigen/Eigen"

using namespace Eigen;

class camera {

protected:
  Matrix3d m3dIntrinsic;
  Vector2d v2dCenter;
  Vector2d v2dFocal;
  Vector2i v2iSize;

public:
  /* Constructors */
  camera();
  camera(MatrixXd mXdIntrinsic);
  camera(Matrix3d m3dIntrinsic);
  camera(double dFocalX, double dFocalY, double dCenterX, double dCenterY, int iSizeX, int iSizeY);

  /* Get Functions */
  Matrix3d getIntrinsic();
  Vector2d getCenter();
  Vector2d getFocal();
  Vector2i getSize();
  double getCenterX();
  double getCenterY();
  double getFocalX();
  double getFocalY();
  double getSizeX();
  double getSizeY();

  /* Set Functions */
  void setIntrinsic(MatrixXd mXdIntrinsic);
  void setIntrinsic(Matrix3d m3dIntrinsic);
  void setIntrinsic(double dFocalX, double dFocalY, double dCenterX, double dCenterY);
  void setIntrinsic(Vector2d v2dFocal, Vector2d v2dCenter);
  void setCenter(double dCenterX, double dCenterY);
  void setCenter(Vector2d v2dCenter);
  void setFocal(double dFocalX, double dFocalY);
  void setFocal(Vector2d v2dFocal);
  void setSize(int iSizeX, int iSizeY);
  void setSize(Vector2i v2iSize);

  /* Projection */
  MatrixXd project();
};

#endif /* POSE_HPP_ */
