/*--------------------------------------------------------------------------
  [Pose Estimation]
  Author: Shehryar Khurshid
  <shehryar87@hotmail.com>

--------------------------------------------------------------------------*/

#include <iostream>
#include <random>
#include <vector>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include "3rdParty/Eigen/Eigen"
#include "Camera.hpp"
#include "Object.hpp"
#include "Pose.hpp"

#define PI 3.1415926535897

using namespace std;
using namespace Eigen;

void startTests();

int main()
{
  // Run tests
  //startTests();

  // Random number seed
  srand(time(NULL));
  default_random_engine generatorI(time(NULL));

  // Define Camera Parameters
  double fx = 800;
  double fy = 800;
  double cx = 320;
  double cy = 240;

  // Intrinsic Matrix
  MatrixXd K = MatrixXd::Identity(3,3);

  K(0,0) = fx;
  K(1,1) = fy;
  K(0,2) = cx;
  K(1,2) = cy;

  // Initialize Camera
  camera newCamera = camera(K);

  // Generate random object
  object newObject;
  newObject.generateObject(10,10,10,10);

  // Generate random pose
  pose newPose;
  newPose.generatePose(0,0,0,0,0,40,PI,PI,PI,15,15,15);

  // Project to image
  image newImage;
  newImage.generateImage(newObject, newCamera, newPose);

  // Perturb Image Points
  newImage.perturbImage(generatorI, 0, 0);

  pose se3initOI = initGuessOI(newObject, newImage, newCamera);

  cout << newPose.getPoseMatrix() << endl << endl;
  cout << se3initOI.getPoseMatrix() << endl << endl;

  return 0;
}
