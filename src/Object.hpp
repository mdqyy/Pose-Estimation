/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#ifndef OBJECT_HPP_
#define OBJECT_HPP_

#include <iostream>
#include "3rdParty/Eigen/Eigen"

using namespace Eigen;

class object {

protected:
  /* 4xn matrix of homogenous 3-D points in object coordinate system */
  MatrixXd mxdObject;

public:
  /* Constructors */
  object();
  object(MatrixXd mxdObject);

  /* Get Functions */
  MatrixXd getObjectPoints();
  unsigned getnRows();
  unsigned getnCols();

  /* Set Functions */
  void setObjectPoints(MatrixXd mxdObject);

  /* Functions */
  void generateObject(double uidimX, double uidimY, double uidimZ, unsigned uinPts);
};

#endif /* OBJECT_HPP_ */
