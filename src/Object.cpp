/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#include <iostream>
#include "3rdParty/Eigen/Eigen"
#include "Object.hpp"
#include <ctime>

using namespace std;
using namespace Eigen;

object::object()
{
  /* Defaut constructor */
}

object::object(MatrixXd mxdObject)
{
  this->mxdObject = mxdObject;
}

MatrixXd object::getObjectPoints()
{
  return this->mxdObject;
}

unsigned object::getnRows()
{
  return unsigned(this->mxdObject.rows());
}

unsigned object::getnCols()
{
  return unsigned(this->mxdObject.cols());
}

void object::setObjectPoints(MatrixXd mxdObject)
{
  this->mxdObject = mxdObject;
}

void object::generateObject(double uidimX, double uidimY, double uidimZ, unsigned uinPts)
{
  this->mxdObject = MatrixXd::Ones(4,uinPts);

  this->mxdObject(0,0) = 0;
  this->mxdObject(1,0) = 0;
  this->mxdObject(2,0) = 0;

  for (unsigned i = 1; i < uinPts; i++)
  {
    this->mxdObject(0,i) =  rand()/double(RAND_MAX) * uidimX;
    this->mxdObject(1,i) =  rand()/double(RAND_MAX) * uidimY;
    this->mxdObject(2,i) =  rand()/double(RAND_MAX) * uidimZ;
  }
}
