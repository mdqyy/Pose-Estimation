/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#ifndef IMAGE_HPP_
#define IMAGE_HPP_

#include <iostream>
#include <random>
#include "3rdParty/Eigen/Eigen"


using namespace std;
using namespace Eigen;

class object;
class camera;
class pose;

class image {

protected:
  /* 3xn matrix of homogenous 2-D image points */
  MatrixXd mxdImage;

public:
  /* Constructors */
  image();
  image(MatrixXd mxdImage);

  /* Get Functions */
  MatrixXd getImagePoints();
  unsigned getnRows();
  unsigned getnCols();

  /* Set Functions */
  void setImagePoints(MatrixXd mxdObject);

  /* Functions */
  void generateImage(object mxdObject, camera m3dCamera, pose m34dPose);
  void perturbImage(default_random_engine gI, double dmean, double dstd);
};


#endif /* IMAGE_HPP_ */
