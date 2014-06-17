/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#include <iostream>
#include "3rdParty/Eigen/Eigen"
#include "Camera.hpp"

using namespace std;
using namespace Eigen;

camera::camera()
{
  this->m3dIntrinsic  = Matrix3d::Identity();
  this->v2dCenter(0)  = 0;
  this->v2dCenter(1)  = 0;
  this->v2dFocal(0)   = 1;
  this->v2dFocal(1)   = 1;
  this->v2iSize(0)    = 1;
  this->v2iSize(1)    = 1;
}

camera::camera(MatrixXd mXdIntrinsic)
{
  /* mXdIntrinsic should be a 3x3 matrix */
  if (mXdIntrinsic.rows() != 3 || mXdIntrinsic.cols() != 3)
  {
    /* mXdIntrinsic is not a 3x3 matrix, construct default camera */
    camera();

    /* Print error */
    cout << "Error: camera::camera(mXdIntrinsic), mXdIntrinsic is not a 3x3 matrix" << endl;
    return;
  }

  this->m3dIntrinsic  = mXdIntrinsic;
  this->v2dCenter(0)  = m3dIntrinsic(0,2);
  this->v2dCenter(1)  = m3dIntrinsic(1,2);
  this->v2dFocal(0)   = m3dIntrinsic(0,0);
  this->v2dFocal(1)   = m3dIntrinsic(1,1);

  if (this->v2dCenter(0) == 0)
    this->v2iSize(0) = 640;
  else
    this->v2iSize(0)    = int(this->v2dCenter(0)*2);
  if (this->v2dCenter(1) == 0)
    this->v2iSize(0) = 480;
  else
    this->v2iSize(1)    = int(this->v2dCenter(0)*2);
}

camera::camera(Matrix3d m3dIntrinsic)
{
  this->m3dIntrinsic  = m3dIntrinsic;
  this->v2dCenter(0)  = m3dIntrinsic(0,2);
  this->v2dCenter(1)  = m3dIntrinsic(1,2);
  this->v2dFocal(0)   = m3dIntrinsic(0,0);
  this->v2dFocal(1)   = m3dIntrinsic(1,1);

  if (this->v2dCenter(0) == 0)
    this->v2iSize(0) = 640;
  else
    this->v2iSize(0)    = int(this->v2dCenter(0)*2);
  if (this->v2dCenter(1) == 0)
    this->v2iSize(0) = 480;
  else
    this->v2iSize(1)    = int(this->v2dCenter(0)*2);
}

camera::camera(double dFocalX, double dFocalY, double dCenterX, double dCenterY, int iSizeX, int iSizeY)
{
  this->m3dIntrinsic      = Matrix3d::Identity();
  this->m3dIntrinsic(0,0) = dFocalX;
  this->m3dIntrinsic(1,1) = dFocalY;
  this->m3dIntrinsic(0,2) = dCenterX;
  this->m3dIntrinsic(1,2) = dCenterY;
  this->v2dCenter(0)      = dCenterX;
  this->v2dCenter(1)      = dCenterY;
  this->v2dFocal(0)       = dFocalX;
  this->v2dFocal(1)       = dFocalY;
  this->v2iSize(0)        = iSizeX;
  this->v2iSize(1)        = iSizeY;
}

Matrix3d camera::getIntrinsic()
{
  return this->m3dIntrinsic;
}
