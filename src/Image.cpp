/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#include <iostream>
#include "3rdParty/Eigen/Eigen"
#include "Image.hpp"
#include "Pose.hpp"
#include "Camera.hpp"
#include "Object.hpp"

using namespace std;
using namespace Eigen;

image::image()
{
  /* Defaut constructor */
}

image::image(MatrixXd mxdImage)
{
  this->mxdImage = mxdImage;
}

MatrixXd image::getImagePoints()
{
  return this->mxdImage;
}

unsigned image::getnRows()
{
  return unsigned(this->mxdImage.rows());

}
unsigned image::getnCols()
{
  return unsigned(this->mxdImage.cols());
}

void image::setImagePoints(MatrixXd mxdImage)
{
  this->mxdImage = mxdImage;
}

void image::generateImage(object mxdObject, camera m3dCamera, pose m34dPose)
{
  this->mxdImage = m3dCamera.getIntrinsic() * m34dPose.getPoseMatrix() * mxdObject.getObjectPoints();

  for (unsigned i = 0; i < this->mxdImage.cols(); i++)
  {
    this->mxdImage(0,i) = round(this->mxdImage(0,i)/this->mxdImage(2,i));
    this->mxdImage(1,i) = round(this->mxdImage(1,i)/this->mxdImage(2,i));
    this->mxdImage(2,i) = round(this->mxdImage(2,i)/this->mxdImage(2,i));
  }
}

void image::perturbImage(default_random_engine gI, double dmean, double dstd)
{
  normal_distribution<double> ndistImg(dmean, dstd);

  for (unsigned i = 0; i < this->mxdImage.cols(); i++)
  {
    this->mxdImage(0,i) = round(this->mxdImage(0,i) + ndistImg(gI));
    this->mxdImage(1,i) = round(this->mxdImage(1,i) + ndistImg(gI));
  }
}
