/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#include <iostream>
#include <ctime>
#include "3rdParty/Eigen/Eigen"
#include "Pose.hpp"

#define PI 3.1415926535897

using namespace std;
using namespace Eigen;

inline double SIGN(double x) {return (x >= 0.0) ? 1.0 : -1.0;}
inline double NORM(double a, double b, double c, double d) {return sqrt(a * a + b * b + c * c + d * d);}

/* Constructors */
rotation::rotation()
{
  this->m3dRotation = Matrix3d::Identity();
}

rotation::rotation(Matrix3d m3dRotation)
{
  this->m3dRotation = m3dRotation;
}

rotation::rotation(Vector3d v3dRotation)
{
  double dax = v3dRotation(0);
  double day = v3dRotation(1);
  double daz = v3dRotation(2);

  double dsx = sin(dax);
  double dcx = cos(dax);
  double dsy = sin(day);
  double dcy = cos(day);
  double dsz = sin(daz);
  double dcz = cos(daz);

  Matrix3d m3dRx = Matrix3d::Identity();
  Matrix3d m3dRy = Matrix3d::Identity();
  Matrix3d m3dRz = Matrix3d::Identity();

  m3dRx(1,1) = dcx;
  m3dRx(1,2) = -1*dsx;
  m3dRx(2,1) = dsx;
  m3dRx(2,2) = dcx;

  m3dRy(0,0) = dcy;
  m3dRy(0,2) = dsy;
  m3dRy(2,0) = -1*dsy;
  m3dRy(2,2) = dcy;

  m3dRz(0,0) = dcz;
  m3dRz(0,1) = -1*dsz;
  m3dRz(1,0) = dsz;
  m3dRz(1,1) = dcz;

  this->m3dRotation = m3dRz * m3dRy * m3dRx;
}

rotation::rotation(double dax, double day, double daz)
{
  double dsx = sin(dax);
  double dcx = cos(dax);
  double dsy = sin(day);
  double dcy = cos(day);
  double dsz = sin(daz);
  double dcz = cos(daz);

  Matrix3d m3dRx = Matrix3d::Identity();
  Matrix3d m3dRy = Matrix3d::Identity();
  Matrix3d m3dRz = Matrix3d::Identity();

  m3dRx(1,1) = dcx;
  m3dRx(1,2) = -1*dsx;
  m3dRx(2,1) = dsx;
  m3dRx(2,2) = dcx;

  m3dRy(0,0) = dcy;
  m3dRy(0,2) = dsy;
  m3dRy(2,0) = -1*dsy;
  m3dRy(2,2) = dcy;

  m3dRz(0,0) = dcz;
  m3dRz(0,1) = -1*dsz;
  m3dRz(1,0) = dsz;
  m3dRz(1,1) = dcz;

  this->m3dRotation = m3dRz * m3dRy * m3dRx;
}

/* Get Functions */
Matrix3d rotation::getRotationMatrix(void)
{
  return this->m3dRotation;
}

Vector3d rotation::getRotationEuler(void)
{
  double day = atan2(-1*this->m3dRotation(2,0), sqrt(this->m3dRotation(0,0) * this->m3dRotation(0,0) + this->m3dRotation(1,0) * this->m3dRotation(1,0)));
  double daz = atan2(this->m3dRotation(1,0)/cos(day), this->m3dRotation(0,0)/cos(day));
  double dax = atan2(this->m3dRotation(2,1)/cos(day), this->m3dRotation(2,2)/cos(day));

  Vector3d v3dRotation;
  v3dRotation(0) = dax;
  v3dRotation(1) = day;
  v3dRotation(2) = daz;

  return v3dRotation;
}

/* Set Functions */
void rotation::setRotation(Matrix3d m3dRotation)
{
  this->m3dRotation = m3dRotation;
}

void rotation::setRotation(Vector3d v3dRotation)
{
  double dax = v3dRotation(0);
  double day = v3dRotation(1);
  double daz = v3dRotation(2);

  double dsx = sin(dax);
  double dcx = cos(dax);
  double dsy = sin(day);
  double dcy = cos(day);
  double dsz = sin(daz);
  double dcz = cos(daz);

  Matrix3d m3dRx = Matrix3d::Identity();
  Matrix3d m3dRy = Matrix3d::Identity();
  Matrix3d m3dRz = Matrix3d::Identity();

  m3dRx(1,1) = dcx;
  m3dRx(1,2) = -1*dsx;
  m3dRx(2,1) = dsx;
  m3dRx(2,2) = dcx;

  m3dRy(0,0) = dcy;
  m3dRy(0,2) = dsy;
  m3dRy(2,0) = -1*dsy;
  m3dRy(2,2) = dcy;

  m3dRz(0,0) = dcz;
  m3dRz(0,1) = -1*dsz;
  m3dRz(1,0) = dsz;
  m3dRz(1,1) = dcz;

  this->m3dRotation = m3dRz * m3dRy * m3dRx;
}

void rotation::setRotation(double dax, double day, double daz)
{
  double dsx = sin(dax);
  double dcx = cos(dax);
  double dsy = sin(day);
  double dcy = cos(day);
  double dsz = sin(daz);
  double dcz = cos(daz);

  Matrix3d m3dRx = Matrix3d::Identity();
  Matrix3d m3dRy = Matrix3d::Identity();
  Matrix3d m3dRz = Matrix3d::Identity();

  m3dRx(1,1) = dcx;
  m3dRx(1,2) = -1*dsx;
  m3dRx(2,1) = dsx;
  m3dRx(2,2) = dcx;

  m3dRy(0,0) = dcy;
  m3dRy(0,2) = dsy;
  m3dRy(2,0) = -1*dsy;
  m3dRy(2,2) = dcy;

  m3dRz(0,0) = dcz;
  m3dRz(0,1) = -1*dsz;
  m3dRz(1,0) = dsz;
  m3dRz(1,1) = dcz;

  this->m3dRotation = m3dRz * m3dRy * m3dRx;
}

pose::pose()
{
  this->mxdPose = MatrixXd::Zero(3,4);
  this->mxdPose(0,0) = 1;
  this->mxdPose(1,1) = 1;
  this->mxdPose(2,2) = 1;

  this->v3dTranslation = Vector3d::Zero();

  this->uiIterations = 0;
  this->dreprojError = 0.0;
  this->dtime = 0.0;
}

pose::pose(MatrixXd mxdPose)
{
  this->mxdPose = MatrixXd::Zero(3,4);
  this->mxdPose(0,0) = 1;
  this->mxdPose(1,1) = 1;
  this->mxdPose(2,2) = 1;

  this->v3dTranslation = Vector3d::Zero();

  this->mxdPose = mxdPose;

  this->uiIterations = 0;
  this->dreprojError = 0.0;
  this->dtime = 0.0;
}

pose::pose(Matrix3d m3dRotation, Vector3d v3dTranslation)
{
  this->mxdPose = MatrixXd::Zero(3,4);
  this->mxdPose(0,0) = 1;
  this->mxdPose(1,1) = 1;
  this->mxdPose(2,2) = 1;

  this->v3dTranslation = Vector3d::Zero();

  this->mxdPose(0,0) = m3dRotation(0,0);
  this->mxdPose(0,1) = m3dRotation(0,1);
  this->mxdPose(0,2) = m3dRotation(0,2);
  this->mxdPose(1,0) = m3dRotation(1,0);
  this->mxdPose(1,1) = m3dRotation(1,1);
  this->mxdPose(1,2) = m3dRotation(1,2);
  this->mxdPose(2,0) = m3dRotation(2,0);
  this->mxdPose(2,1) = m3dRotation(2,1);
  this->mxdPose(2,2) = m3dRotation(2,2);
  this->mxdPose(0,3) = v3dTranslation(0);
  this->mxdPose(1,3) = v3dTranslation(1);
  this->mxdPose(2,3) = v3dTranslation(2);

  this->m3dRotation.setRotation(m3dRotation);

  this->v3dTranslation = v3dTranslation;

  this->uiIterations = 0;
  this->dreprojError = 0.0;
  this->dtime = 0.0;
}

pose::pose(Vector3d v3dEulerAngles, Vector3d v3dTranslation)
{
  this->mxdPose = MatrixXd::Zero(3,4);
  this->mxdPose(0,0) = 1;
  this->mxdPose(1,1) = 1;
  this->mxdPose(2,2) = 1;

  this->v3dTranslation = Vector3d::Zero();

  this->m3dRotation.setRotation(v3dEulerAngles);

  this->mxdPose(0,0) = this->m3dRotation.getRotationMatrix()(0,0);
  this->mxdPose(0,1) = this->m3dRotation.getRotationMatrix()(0,1);
  this->mxdPose(0,2) = this->m3dRotation.getRotationMatrix()(0,2);
  this->mxdPose(1,0) = this->m3dRotation.getRotationMatrix()(1,0);
  this->mxdPose(1,1) = this->m3dRotation.getRotationMatrix()(1,1);
  this->mxdPose(1,2) = this->m3dRotation.getRotationMatrix()(1,2);
  this->mxdPose(2,0) = this->m3dRotation.getRotationMatrix()(2,0);
  this->mxdPose(2,1) = this->m3dRotation.getRotationMatrix()(2,1);
  this->mxdPose(2,2) = this->m3dRotation.getRotationMatrix()(2,2);
  this->mxdPose(0,3) = v3dTranslation(0);
  this->mxdPose(1,3) = v3dTranslation(1);
  this->mxdPose(2,3) = v3dTranslation(2);

  this->v3dTranslation = v3dTranslation;

  this->uiIterations = 0;
  this->dreprojError = 0.0;
  this->dtime = 0.0;
}

pose::pose(VectorXd v6dPose)
{
  Vector3d v3dEulerAngles = v6dPose.head(3);
  Vector3d v3dTranslation = v6dPose.tail(3);

  this->mxdPose = MatrixXd::Zero(3,4);
  this->mxdPose(0,0) = 1;
  this->mxdPose(1,1) = 1;
  this->mxdPose(2,2) = 1;

  this->v3dTranslation = Vector3d::Zero();

  this->m3dRotation.setRotation(v3dEulerAngles);

  this->mxdPose(0,0) = this->m3dRotation.getRotationMatrix()(0,0);
  this->mxdPose(0,1) = this->m3dRotation.getRotationMatrix()(0,1);
  this->mxdPose(0,2) = this->m3dRotation.getRotationMatrix()(0,2);
  this->mxdPose(1,0) = this->m3dRotation.getRotationMatrix()(1,0);
  this->mxdPose(1,1) = this->m3dRotation.getRotationMatrix()(1,1);
  this->mxdPose(1,2) = this->m3dRotation.getRotationMatrix()(1,2);
  this->mxdPose(2,0) = this->m3dRotation.getRotationMatrix()(2,0);
  this->mxdPose(2,1) = this->m3dRotation.getRotationMatrix()(2,1);
  this->mxdPose(2,2) = this->m3dRotation.getRotationMatrix()(2,2);
  this->mxdPose(0,3) = v3dTranslation(0);
  this->mxdPose(1,3) = v3dTranslation(1);
  this->mxdPose(2,3) = v3dTranslation(2);

  this->v3dTranslation = v3dTranslation;

  this->uiIterations = 0;
  this->dreprojError = 0.0;
  this->dtime = 0.0;
}

pose::pose(double dax, double day, double daz, Vector3d v3dTranslation)
{
  this->mxdPose = MatrixXd::Zero(3,4);
  this->mxdPose(0,0) = 1;
  this->mxdPose(1,1) = 1;
  this->mxdPose(2,2) = 1;

  this->v3dTranslation = Vector3d::Zero();

  this->m3dRotation.setRotation(dax, day, daz);

  this->mxdPose(0,0) = this->m3dRotation.getRotationMatrix()(0,0);
  this->mxdPose(0,1) = this->m3dRotation.getRotationMatrix()(0,1);
  this->mxdPose(0,2) = this->m3dRotation.getRotationMatrix()(0,2);
  this->mxdPose(1,0) = this->m3dRotation.getRotationMatrix()(1,0);
  this->mxdPose(1,1) = this->m3dRotation.getRotationMatrix()(1,1);
  this->mxdPose(1,2) = this->m3dRotation.getRotationMatrix()(1,2);
  this->mxdPose(2,0) = this->m3dRotation.getRotationMatrix()(2,0);
  this->mxdPose(2,1) = this->m3dRotation.getRotationMatrix()(2,1);
  this->mxdPose(2,2) = this->m3dRotation.getRotationMatrix()(2,2);
  this->mxdPose(0,3) = v3dTranslation(0);
  this->mxdPose(1,3) = v3dTranslation(1);
  this->mxdPose(2,3) = v3dTranslation(2);

  this->v3dTranslation = v3dTranslation;

  this->uiIterations = 0;
  this->dreprojError = 0.0;
  this->dtime = 0.0;
}

pose::pose(double dax, double day, double daz, double dtx, double dty, double dtz)
{
  this->mxdPose = MatrixXd::Zero(3,4);
  this->mxdPose(0,0) = 1;
  this->mxdPose(1,1) = 1;
  this->mxdPose(2,2) = 1;

  this->v3dTranslation = Vector3d::Zero();

  this->m3dRotation.setRotation(dax, day, daz);

  this->mxdPose(0,0) = this->m3dRotation.getRotationMatrix()(0,0);
  this->mxdPose(0,1) = this->m3dRotation.getRotationMatrix()(0,1);
  this->mxdPose(0,2) = this->m3dRotation.getRotationMatrix()(0,2);
  this->mxdPose(1,0) = this->m3dRotation.getRotationMatrix()(1,0);
  this->mxdPose(1,1) = this->m3dRotation.getRotationMatrix()(1,1);
  this->mxdPose(1,2) = this->m3dRotation.getRotationMatrix()(1,2);
  this->mxdPose(2,0) = this->m3dRotation.getRotationMatrix()(2,0);
  this->mxdPose(2,1) = this->m3dRotation.getRotationMatrix()(2,1);
  this->mxdPose(2,2) = this->m3dRotation.getRotationMatrix()(2,2);
  this->mxdPose(0,3) = dtx;
  this->mxdPose(1,3) = dty;
  this->mxdPose(2,3) = dtz;

  this->v3dTranslation(0) = dtx;
  this->v3dTranslation(1) = dty;
  this->v3dTranslation(2) = dtz;

  this->uiIterations = 0;
  this->dreprojError = 0.0;
  this->dtime = 0.0;
}

MatrixXd pose::getPoseMatrix(void)
{
  return this->mxdPose;
}

VectorXd pose::getPoseVector(void)
{
  Vector3d v3dRotation = this->m3dRotation.getRotationEuler();

  VectorXd v6dPose(6);
  v6dPose(0) = v3dRotation(0);
  v6dPose(1) = v3dRotation(1);
  v6dPose(2) = v3dRotation(2);
  v6dPose(3) = this->v3dTranslation(0);
  v6dPose(4) = this->v3dTranslation(1);
  v6dPose(5) = this->v3dTranslation(2);

  return v6dPose;
}

Matrix3d pose::getRotationMatrix(void)
{
  return this->m3dRotation.getRotationMatrix();
}

Vector3d pose::getRotationEuler(void)
{
  return this->m3dRotation.getRotationEuler();
}

Vector4d pose::getRotationQuaternion(void)
{
  Vector4d v4dquaternion;
  Matrix3d m3dR = this->getRotationMatrix();

  double r11 = m3dR(0,0);
  double r12 = m3dR(0,1);
  double r13 = m3dR(0,2);
  double r21 = m3dR(1,0);
  double r22 = m3dR(1,1);
  double r23 = m3dR(1,2);
  double r31 = m3dR(2,0);
  double r32 = m3dR(2,1);
  double r33 = m3dR(2,2);

  double q0 = (r11 + r22 + r33 + 1.0)/4.0;
  double q1 = (r11 - r22 - r33 + 1.0)/4.0;
  double q2 = (-r11 + r22 - r33 + 1.0)/4.0;
  double q3 = (-r11 - r22 + r33 + 1.0)/4.0;

  if(q0 < 0.0) q0 = 0.0;
  if(q1 < 0.0) q1 = 0.0;
  if(q2 < 0.0) q2 = 0.0;
  if(q3 < 0.0) q3 = 0.0;

  q0 = sqrt(q0);
  q1 = sqrt(q1);
  q2 = sqrt(q2);
  q3 = sqrt(q3);

  if(q0 >= q1 && q0 >= q2 && q0 >= q3)
  {
      q0 *= 1.0;
      q1 *= SIGN(r32 - r23);
      q2 *= SIGN(r13 - r31);
      q3 *= SIGN(r21 - r12);
  }

  else if(q1 >= q0 && q1 >= q2 && q1 >= q3)
  {
      q0 *= SIGN(r32 - r23);
      q1 *= 1.0;
      q2 *= SIGN(r21 + r12);
      q3 *= SIGN(r13 + r31);
  }

  else if(q2 >= q0 && q2 >= q1 && q2 >= q3)
  {
      q0 *= SIGN(r13 - r31);
      q1 *= SIGN(r21 + r12);
      q2 *= 1.0;
      q3 *= SIGN(r32 + r23);
  }

  else if(q3 >= q0 && q3 >= q1 && q3 >= q2)
  {
      q0 *= SIGN(r21 - r12);
      q1 *= SIGN(r31 + r13);
      q2 *= SIGN(r32 + r23);
      q3 *= 1.0;
  }

  else
  {
      cout << "Coding Error" << endl;
  }

  double r = NORM(q0, q1, q2, q3);
  q0 /= r;
  q1 /= r;
  q2 /= r;
  q3 /= r;

  v4dquaternion(0) = q0;
  v4dquaternion(1) = q1;
  v4dquaternion(2) = q2;
  v4dquaternion(3) = q3;

  return v4dquaternion;
}

Vector3d pose::getTranstalion(void)
{
  return this->v3dTranslation;
}

unsigned pose::getIterations(void)
{
  return this->uiIterations;
}

double pose::getReprojError(void)
{
  return this->dreprojError;
}

double pose::getTime(void)
{
  return this->dtime;
}

void pose::setPose(MatrixXd mxdPose)
{
  this->mxdPose = mxdPose;
}

void pose::setPose(Matrix3d m3dRotation, Vector3d v3dTranslation)
{
  this->m3dRotation.setRotation(m3dRotation);
  this->v3dTranslation = v3dTranslation;

  this->mxdPose(0,0) = this->m3dRotation.getRotationMatrix()(0,0);
  this->mxdPose(0,1) = this->m3dRotation.getRotationMatrix()(0,1);
  this->mxdPose(0,2) = this->m3dRotation.getRotationMatrix()(0,2);
  this->mxdPose(1,0) = this->m3dRotation.getRotationMatrix()(1,0);
  this->mxdPose(1,1) = this->m3dRotation.getRotationMatrix()(1,1);
  this->mxdPose(1,2) = this->m3dRotation.getRotationMatrix()(1,2);
  this->mxdPose(2,0) = this->m3dRotation.getRotationMatrix()(2,0);
  this->mxdPose(2,1) = this->m3dRotation.getRotationMatrix()(2,1);
  this->mxdPose(2,2) = this->m3dRotation.getRotationMatrix()(2,2);
  this->mxdPose(0,3) = v3dTranslation(0);
  this->mxdPose(1,3) = v3dTranslation(1);
  this->mxdPose(2,3) = v3dTranslation(2);
}

void pose::setIterations(unsigned uinIterations)
{
  this->uiIterations = uinIterations;
}

void pose::setReprojError(double drprojError)
{
  this->dreprojError = drprojError;
}

void pose::setTime(double deTime)
{
  this->dtime = deTime;
}

void pose::generatePose(double ax, double ay, double az,\
    double tx, double ty, double tz,\
    double dax, double day, double daz,\
    double dtx, double dty, double dtz)
{
  Vector3d v3dRVec;
  Vector3d v3dtVec;

  v3dRVec(0) = ax + dax * (rand()/double(RAND_MAX)*2 - 1);
  v3dRVec(1) = ay + day * (rand()/double(RAND_MAX)*2 - 1);
  v3dRVec(2) = az + daz * (rand()/double(RAND_MAX)*2 - 1);
  v3dtVec(0) = tx + dtx * (rand()/double(RAND_MAX)*2 - 1);
  v3dtVec(1) = ty + dty * (rand()/double(RAND_MAX)*2 - 1);
  v3dtVec(2) = tz + dtz * (rand()/double(RAND_MAX)*2 - 1);

  this->v3dTranslation = Vector3d::Zero();

  this->m3dRotation.setRotation(v3dRVec);

  this->mxdPose(0,0) = this->m3dRotation.getRotationMatrix()(0,0);
  this->mxdPose(0,1) = this->m3dRotation.getRotationMatrix()(0,1);
  this->mxdPose(0,2) = this->m3dRotation.getRotationMatrix()(0,2);
  this->mxdPose(1,0) = this->m3dRotation.getRotationMatrix()(1,0);
  this->mxdPose(1,1) = this->m3dRotation.getRotationMatrix()(1,1);
  this->mxdPose(1,2) = this->m3dRotation.getRotationMatrix()(1,2);
  this->mxdPose(2,0) = this->m3dRotation.getRotationMatrix()(2,0);
  this->mxdPose(2,1) = this->m3dRotation.getRotationMatrix()(2,1);
  this->mxdPose(2,2) = this->m3dRotation.getRotationMatrix()(2,2);
  this->mxdPose(0,3) = v3dtVec(0);
  this->mxdPose(1,3) = v3dtVec(1);
  this->mxdPose(2,3) = v3dtVec(2);

  this->v3dTranslation = v3dtVec;
}

double quaternionError(Vector4d v4dQ1, Vector4d v4dQ2)
{
  double derror1, derror2;

  derror1 = (v4dQ1-v4dQ2).norm()/v4dQ2.norm()*100;
  derror2 = (v4dQ1+v4dQ2).norm()/v4dQ2.norm()*100;

  if (derror1 < derror2)
    return derror1;

  else
    return derror2;
}

double translationError(Vector3d v3dTr1, Vector3d v3dTr2)
{
  double derror = (v3dTr1-v3dTr2).norm()/v3dTr1.norm()*100;
  return derror;
}

double rotationError(Matrix3d m3dR1, Matrix3d m3dR2)
{
  Matrix3d m3dDiff = m3dR1 * m3dR2.transpose();

  double diff = (m3dDiff.trace() - 1)/2;

  if (diff >= 1)
  {
    diff = 0.9999999999;
  }

  if (diff <= -1)
  {
    diff = -0.9999999999;
  }

  double error = acos(diff) * 180/PI;

  return error;
}
