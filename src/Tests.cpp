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

inline double findMean(vector<double> vec)
{
  double sum = accumulate(vec.begin(), vec.end(), 0.0);
  return sum / vec.size();
}

inline double findStdDev(vector<double> vec)
{
  double mean = findMean(vec);

  double sq_sum = inner_product(vec.begin(), vec.end(), vec.begin(), 0.0);
  return sqrt(sq_sum / vec.size() - mean * mean);
}

inline double findMin(vector<double> vec)
{
  vector<double>::iterator minIdx = min_element(vec.begin(), vec.end());
  return *minIdx;
}

inline double findMax(vector<double> vec)
{
  vector<double>::iterator maxIdx = max_element(vec.begin(), vec.end());
  return *maxIdx;
}

inline vector<double> findQuartiles(vector<double> vec)
{
  typedef vector<double>::size_type vec_sz;

  vec_sz size = vec.size();

  sort(vec.begin(), vec.end());

  vec_sz mid = size/2;
  vec_sz midL = size/4;
  vec_sz midU = 3*size/4;

  double firstQuartile, median, thirdQuartile;

  if (size % 2 == 0)
  {
    firstQuartile = vec[midL];
    median = (vec[mid] + vec[mid-1])/2;
    thirdQuartile = vec[midU];
  }

  else
  {
    firstQuartile = vec[midL];
    median = vec[mid];
    thirdQuartile = vec[midU];
  }

  vector<double> Quartiles;
  Quartiles.push_back(firstQuartile);
  Quartiles.push_back(median);
  Quartiles.push_back(thirdQuartile);

  return Quartiles;
}

inline double findIQR(double dQ1, double dQ3)
{
  return (dQ3-dQ1);
}

inline vector<double> findWhiskers(double dQ1, double dQ3)
{
  double dIQR = (dQ3-dQ1);
  vector<double> whiskers;

  whiskers.push_back(dQ1-1.5*dIQR);
  whiskers.push_back(dQ3+1.5*dIQR);

  return whiskers;
}

inline double findnOutliers(vector<double> vec, double lower, double upper)
{
  double outCount = 0;

  for (vector<double>::iterator it=vec.begin(); it!=vec.end(); ++it)
  {
    if (*it >= upper || *it <= lower)
    {
      outCount += 1;
    }
  }

  return outCount;
}

inline double findMin2(vector<double> vec)
{
  vector<double> Quartiles = findQuartiles(vec);
  double dQ1 = Quartiles[0];
  double dQ3 = Quartiles[2];
  double dIQR = dQ3 - dQ1;

  double lower = dQ1 - 1.5*dIQR;

  double min = 1e10;

  for (vector<double>::iterator it=vec.begin(); it!=vec.end(); ++it)
  {
    if (*it < min && *it >= lower)
    {
      min = *it;
    }
  }

  return min;
}

inline double findMax2(vector<double> vec)
{
  vector<double> Quartiles = findQuartiles(vec);
  double dQ1 = Quartiles[0];
  double dQ3 = Quartiles[2];
  double dIQR = dQ3 - dQ1;

  double upper = dQ3 + 1.5*dIQR;

  double max = 1e-10;

  for (vector<double>::iterator it=vec.begin(); it!=vec.end(); ++it)
  {
    if (*it > max && *it <= upper)
    {
      max = *it;
    }
  }

  return max;
}

void startTests()
{
  // Test A. No of Points
  //       A.1. Speed (nPts = 6-150, var = 5, outliers = 0 %)
  //       A.2. Accuracy (nPts = 6-50, var = 10, outliers = 0 %))
  //
  // Test B. Noise
  //       B.1. Speed (nPts = 6, var = 0-10, outliers = 0 %)
  //       B.2. Accuracy (nPts = 6, var = 0-10, outliers = 0 %)
  //
  // Test C. Outliers
  //       C.1. Speed (nPts = 100, var = 5, outliers = 0-100 %)
  //       C.2. Accuracy (nPts = 100, var = 5, outliers = 0-100 %)
  //
  // Test D. Distance from the camera
  //       D.1. Speed (nPts = 6, var = 0, outliers = 0 %, z=50-0)
  //       D.2. Accuracy (nPts = 6, var = 0, outliers = 0 %, z=50-0)
  //
  // Test E. Distance of the initial guess from the true pose
  //       E.1. Speed (nPts = 6, var = 0, outliers = 0 %)
  //       E.2. Accuracy (nPts = 6, var = 0, outliers = 0 %)

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

  unsigned uinPtsL = 6;
  unsigned uinPtsU = 150;
  unsigned uinTrials = 5000;

  vector<double> tDLTMean;
  vector<double> tDLTMedian;
  vector<double> tDLTStdDev;
  vector<double> tDLTFQuartile;
  vector<double> tDLTTQuartile;
  vector<double> tDLTMin;
  vector<double> tDLTMax;
  vector<double> tDLTSR;

  vector<double> tEPNPMean;
  vector<double> tEPNPMedian;
  vector<double> tEPNPStdDev;
  vector<double> tEPNPFQuartile;
  vector<double> tEPNPTQuartile;
  vector<double> tEPNPMin;
  vector<double> tEPNPMax;
  vector<double> tEPNPSR;

  vector<double> tPOSITMean;
  vector<double> tPOSITMedian;
  vector<double> tPOSITStdDev;
  vector<double> tPOSITFQuartile;
  vector<double> tPOSITTQuartile;
  vector<double> tPOSITMin;
  vector<double> tPOSITMax;
  vector<double> tPOSITSR;

  vector<double> tOIMean;
  vector<double> tOIMedian;
  vector<double> tOIStdDev;
  vector<double> tOIFQuartile;
  vector<double> tOITQuartile;
  vector<double> tOIMin;
  vector<double> tOIMax;
  vector<double> tOISR;

  vector<double> tLMMean;
  vector<double> tLMMedian;
  vector<double> tLMStdDev;
  vector<double> tLMFQuartile;
  vector<double> tLMTQuartile;
  vector<double> tLMMin;
  vector<double> tLMMax;
  vector<double> tLMSR;

  double progressN = 0;
  double progressL = 0;
  double tIterations = (uinPtsU-uinPtsL+1)*uinTrials;
  double nIteration = 0;

  pose se3initPose(0,0,0,0,0,40);
  pose se3PoseDLT;
  pose se3PoseEPNP;
  pose se3PosePOSIT;
  pose se3PoseOI;
  pose se3PoseLM;

  double rotErrDLT;
  double trErrDLT;

  double rotErrEPNP;
  double trErrEPNP;

  double rotErrPOSIT;
  double trErrPOSIT;

  double rotErrOI;
  double trErrOI;

  double rotErrLM;
  double trErrLM;

  timeval t1, t2;
  double elapsedTime;

  cout << endl << "Running Test A. No. of points" << endl << endl;
  cout << " - A.1. Speed (nPts = 6-150, var = 5, outliers = 0 %) " << endl;

  for (unsigned nPt=uinPtsL; nPt<=uinPtsU; nPt++)
  {
    vector<double> tDLT;
    vector<double> tEPNP;
    vector<double> tPOSIT;
    vector<double> tOI;
    vector<double> tLM;

    double successDLT = 0;
    double successEPNP = 0;
    double successPOSIT = 0;
    double successLM = 0;
    double successOI = 0;

    for (unsigned nTr=0; nTr<uinTrials; nTr++)
    {
      // Generate random object
      object newObject;
      newObject.generateObject(25,25,25,nPt);

      // Generate random pose
      pose newPose;
      newPose.generatePose(0,0,0,0,0,40,PI,PI,PI,15,15,15);

      // Project to image
      image newImage;
      newImage.generateImage(newObject, newCamera, newPose);

      // Perturb Image Points
      newImage.perturbImage(generatorI, 0, 5);

      // DLT
      gettimeofday(&t1, NULL);
      se3PoseDLT = normalizedDLT(newObject, newImage, newCamera);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrDLT = rotationError(newPose.getRotationMatrix(), se3PoseDLT.getRotationMatrix());
      trErrDLT = translationError(newPose.getTranstalion(), se3PoseDLT.getTranstalion());

      if (rotErrDLT <= 150 && trErrDLT < 100)
      {
        tDLT.push_back(elapsedTime);
        successDLT += 1;
      }

      // EPNP
      gettimeofday(&t1, NULL);
      se3PoseEPNP = EPNP(newObject, newImage, newCamera);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrEPNP = rotationError(newPose.getRotationMatrix(), se3PoseEPNP.getRotationMatrix());
      trErrEPNP = translationError(newPose.getTranstalion(), se3PoseEPNP.getTranstalion());

      if (rotErrEPNP <= 150 && trErrEPNP < 100)
      {
        tEPNP.push_back(elapsedTime);
        successEPNP += 1;
      }

      // POSIT
      gettimeofday(&t1, NULL);
      se3PosePOSIT = POSIT(newObject, newImage, newCamera, 100);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrPOSIT = rotationError(newPose.getRotationMatrix(), se3PosePOSIT.getRotationMatrix());
      trErrPOSIT = translationError(newPose.getTranstalion(), se3PosePOSIT.getTranstalion());

      if (rotErrPOSIT <= 150 && trErrPOSIT < 100)
      {
        tPOSIT.push_back(elapsedTime);
        successPOSIT += 1;
      }

      // OI
      gettimeofday(&t1, NULL);
      se3PoseOI = OI(newObject, newImage, newCamera, se3initPose, 100);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrOI = rotationError(newPose.getRotationMatrix(), se3PoseOI.getRotationMatrix());
      trErrOI = translationError(newPose.getTranstalion(), se3PoseOI.getTranstalion());

      if (rotErrOI <= 150 && trErrOI < 100)
      {
        tOI.push_back(elapsedTime);
        successOI += 1;
      }

      // LM
      gettimeofday(&t1, NULL);
      se3PoseLM = LM(newObject, newImage, newCamera, se3PoseEPNP, 100);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrLM = rotationError(newPose.getRotationMatrix(), se3PoseLM.getRotationMatrix());
      trErrLM = translationError(newPose.getTranstalion(), se3PoseLM.getTranstalion());

      if (rotErrLM <= 150 && trErrLM < 100)
      {
        tLM.push_back(elapsedTime);
        successLM += 1;
      }

      progressN = (double)nIteration/(double)tIterations*100;
      cout << "\r   Progress: " << progressN << " %      ";
      nIteration += 1;
    }

    vector<double> tDLTQuartiles = findQuartiles(tDLT);
    tDLTMean.push_back(findMean(tDLT));
    tDLTMedian.push_back(tDLTQuartiles[1]);
    tDLTStdDev.push_back(findStdDev(tDLT));
    tDLTFQuartile.push_back(tDLTQuartiles[0]);
    tDLTTQuartile.push_back(tDLTQuartiles[2]);
    tDLTMin.push_back(findMin2(tDLT));
    tDLTMax.push_back(findMax2(tDLT));
    tDLTSR.push_back(findnOutliers(tDLT, tDLTQuartiles[0] - 1.5 * (tDLTQuartiles[2] - tDLTQuartiles[0]), tDLTQuartiles[0] + 1.5 * (tDLTQuartiles[2] - tDLTQuartiles[0]))/uinTrials*100);

    vector<double> tEPNPQuartiles = findQuartiles(tEPNP);
    tEPNPMean.push_back(findMean(tEPNP));
    tEPNPMedian.push_back(tEPNPQuartiles[1]);
    tEPNPStdDev.push_back(findStdDev(tEPNP));
    tEPNPFQuartile.push_back(tEPNPQuartiles[0]);
    tEPNPTQuartile.push_back(tEPNPQuartiles[2]);
    tEPNPMin.push_back(findMin2(tEPNP));
    tEPNPMax.push_back(findMax2(tEPNP));
    tEPNPSR.push_back(findnOutliers(tEPNP, tEPNPQuartiles[0] - 1.5 * (tEPNPQuartiles[2] - tEPNPQuartiles[0]), tEPNPQuartiles[0] + 1.5 * (tEPNPQuartiles[2] - tEPNPQuartiles[0]))/uinTrials*100);

    vector<double> tPOSITQuartiles = findQuartiles(tPOSIT);
    tPOSITMean.push_back(findMean(tPOSIT));
    tPOSITMedian.push_back(tPOSITQuartiles[1]);
    tPOSITStdDev.push_back(findStdDev(tPOSIT));
    tPOSITFQuartile.push_back(tPOSITQuartiles[0]);
    tPOSITTQuartile.push_back(tPOSITQuartiles[2]);
    tPOSITMin.push_back(findMin2(tPOSIT));
    tPOSITMax.push_back(findMax2(tPOSIT));
    tPOSITSR.push_back(findnOutliers(tPOSIT, tPOSITQuartiles[0] - 1.5 * (tPOSITQuartiles[2] - tPOSITQuartiles[0]), tPOSITQuartiles[0] + 1.5 * (tPOSITQuartiles[2] - tPOSITQuartiles[0]))/uinTrials*100);

    vector<double> tOIQuartiles = findQuartiles(tOI);
    tOIMean.push_back(findMean(tOI));
    tOIMedian.push_back(tOIQuartiles[1]);
    tOIStdDev.push_back(findStdDev(tOI));
    tOIFQuartile.push_back(tOIQuartiles[0]);
    tOITQuartile.push_back(tOIQuartiles[2]);
    tOIMin.push_back(findMin2(tOI));
    tOIMax.push_back(findMax2(tOI));
    tOISR.push_back(findnOutliers(tOI, tOIQuartiles[0] - 1.5 * (tOIQuartiles[2] - tOIQuartiles[0]), tOIQuartiles[0] + 1.5 * (tOIQuartiles[2] - tOIQuartiles[0]))/uinTrials*100);

    vector<double> tLMQuartiles = findQuartiles(tLM);
    tLMMean.push_back(findMean(tLM));
    tLMMedian.push_back(tLMQuartiles[1]);
    tLMStdDev.push_back(findStdDev(tLM));
    tLMFQuartile.push_back(tLMQuartiles[0]);
    tLMTQuartile.push_back(tLMQuartiles[2]);
    tLMMin.push_back(findMin2(tLM));
    tLMMax.push_back(findMax2(tLM));
    tLMSR.push_back(findnOutliers(tLM, tLMQuartiles[0] - 1.5 * (tLMQuartiles[2] - tLMQuartiles[0]), tLMQuartiles[0] + 1.5 * (tLMQuartiles[2] - tLMQuartiles[0]))/uinTrials*100);
  }

  ofstream fileA1("plotA1.dat");
  fileA1 << "# plotA1.dat" << endl;
  fileA1 << "# Data file for GNUPLOT" << endl;
  fileA1 << "# Test A. No. of points" << endl;
  fileA1 << "# - A.1. Speed (nPts = 6-150, var = 5, outliers = 0 %)" << endl;
  fileA1 << "# nPts  " << "\t"
         << "u-DLT   " << "\t" << "m-DLT   " << "\t" << "p-DLT   " << "\t" << "1q-DLT  " << "\t" << "3q-DLT  " << "\t" << "mn-DLT  " << "\t" << "mx-DLT  " << "\t" << "SR-DLT  " << "\t"
         << "u-EPN   " << "\t" << "m-EPN   " << "\t" << "p-EPN   " << "\t" << "1q-EPN  " << "\t" << "3q-EPN  " << "\t" << "mn-EPN  " << "\t" << "mx-EPN  " << "\t" << "SR-EPN  " << "\t"
         << "u-POS   " << "\t" << "m-POS   " << "\t" << "p-POS   " << "\t" << "1q-POS  " << "\t" << "3q-POS  " << "\t" << "mn-POS  " << "\t" << "mx-POS  " << "\t" << "SR-POS  " << "\t"
         << "u-OI    " << "\t" << "m-OI    " << "\t" << "p-OI    " << "\t" << "1q-OI   " << "\t" << "3q-OI   " << "\t" << "mn-OI   " << "\t" << "mx-OI   " << "\t" << "SR-OI   " << "\t"
         << "u-LM    " << "\t" << "m-LM    " << "\t" << "p-LM    " << "\t" << "1q-LM   " << "\t" << "3q-LM   " << "\t" << "mn-LM   " << "\t" << "mx-LM   " << "\t" << "SR-LM   " << "\t" << endl;

  for (unsigned i=0; i<=uinPtsU - uinPtsL; i++)
  {
    fileA1 << fixed << setprecision(6) << (double)i+uinPtsL << "\t"
           << tDLTMean[i] << "\t" << tDLTMedian[i] << "\t" << tDLTStdDev[i] << "\t" << tDLTFQuartile[i] << "\t" << tDLTTQuartile[i] << "\t" << tDLTMin[i] << "\t" << tDLTMax[i] << "\t" << tDLTSR[i] << "\t"
           << tEPNPMean[i] << "\t" << tEPNPMedian[i] << "\t" << tEPNPStdDev[i] << "\t" << tEPNPFQuartile[i] << "\t" << tEPNPTQuartile[i] << "\t" << tEPNPMin[i] << "\t" << tEPNPMax[i] << "\t" << tEPNPSR[i] << "\t"
           << tPOSITMean[i] << "\t" << tPOSITMedian[i] << "\t" << tPOSITStdDev[i] << "\t" << tPOSITFQuartile[i] << "\t" << tPOSITTQuartile[i] << "\t" << tPOSITMin[i] << "\t" << tPOSITMax[i] << "\t" << tPOSITSR[i] << "\t"
           << tOIMean[i] << "\t" << tOIMedian[i] << "\t" << tOIStdDev[i] << "\t" << tOIFQuartile[i] << "\t" << tOITQuartile[i] << "\t" << tOIMin[i] << "\t" << tOIMax[i] << "\t" << tOISR[i] << "\t"
           << tLMMean[i] << "\t" << tLMMedian[i] << "\t" << tLMStdDev[i] << "\t" << tLMFQuartile[i] << "\t" << tLMTQuartile[i] << "\t" << tLMMin[i] << "\t" << tLMMax[i] << "\t" << tLMSR[i] << "\t" << endl;

  }

  fileA1.close();
  cout << endl << endl;

  uinPtsL = 6;
  uinPtsU = 50;
  uinTrials = 5000;

  vector<double> rotErrDLTMean;
  vector<double> rotErrDLTMedian;
  vector<double> rotErrDLTStdDev;
  vector<double> rotErrDLTFQuartile;
  vector<double> rotErrDLTTQuartile;
  vector<double> rotErrDLTMin;
  vector<double> rotErrDLTMax;

  vector<double> trErrDLTMean;
  vector<double> trErrDLTMedian;
  vector<double> trErrDLTStdDev;
  vector<double> trErrDLTFQuartile;
  vector<double> trErrDLTTQuartile;
  vector<double> trErrDLTMin;
  vector<double> trErrDLTMax;

  vector<double> DLTSR;

  vector<double> rotErrEPNPMean;
  vector<double> rotErrEPNPMedian;
  vector<double> rotErrEPNPStdDev;
  vector<double> rotErrEPNPFQuartile;
  vector<double> rotErrEPNPTQuartile;
  vector<double> rotErrEPNPMin;
  vector<double> rotErrEPNPMax;

  vector<double> trErrEPNPMean;
  vector<double> trErrEPNPMedian;
  vector<double> trErrEPNPStdDev;
  vector<double> trErrEPNPFQuartile;
  vector<double> trErrEPNPTQuartile;
  vector<double> trErrEPNPMin;
  vector<double> trErrEPNPMax;

  vector<double> EPNPSR;

  vector<double> rotErrPOSITMean;
  vector<double> rotErrPOSITMedian;
  vector<double> rotErrPOSITStdDev;
  vector<double> rotErrPOSITFQuartile;
  vector<double> rotErrPOSITTQuartile;
  vector<double> rotErrPOSITMin;
  vector<double> rotErrPOSITMax;

  vector<double> trErrPOSITMean;
  vector<double> trErrPOSITMedian;
  vector<double> trErrPOSITStdDev;
  vector<double> trErrPOSITFQuartile;
  vector<double> trErrPOSITTQuartile;
  vector<double> trErrPOSITMin;
  vector<double> trErrPOSITMax;

  vector<double> POSITSR;

  vector<double> rotErrOIMean;
  vector<double> rotErrOIMedian;
  vector<double> rotErrOIStdDev;
  vector<double> rotErrOIFQuartile;
  vector<double> rotErrOITQuartile;
  vector<double> rotErrOIMin;
  vector<double> rotErrOIMax;

  vector<double> trErrOIMean;
  vector<double> trErrOIMedian;
  vector<double> trErrOIStdDev;
  vector<double> trErrOIFQuartile;
  vector<double> trErrOITQuartile;
  vector<double> trErrOIMin;
  vector<double> trErrOIMax;

  vector<double> OISR;

  vector<double> rotErrLMMean;
  vector<double> rotErrLMMedian;
  vector<double> rotErrLMStdDev;
  vector<double> rotErrLMFQuartile;
  vector<double> rotErrLMTQuartile;
  vector<double> rotErrLMMin;
  vector<double> rotErrLMMax;

  vector<double> trErrLMMean;
  vector<double> trErrLMMedian;
  vector<double> trErrLMStdDev;
  vector<double> trErrLMFQuartile;
  vector<double> trErrLMTQuartile;
  vector<double> trErrLMMin;
  vector<double> trErrLMMax;

  vector<double> LMSR;

  progressN = 0;
  progressL = 0;
  tIterations = (uinPtsU-uinPtsL+1)*uinTrials;
  nIteration = 0;

  cout << " - A.2. Accuracy (nPts = 6-50, var = 10, outliers = 0 %) " << endl;

  for (unsigned nPt=uinPtsL; nPt<=uinPtsU; nPt++)
  {
    vector<double> rotEDLT;
    vector<double> rotEEPNP;
    vector<double> rotEPOSIT;
    vector<double> rotEOI;
    vector<double> rotELM;

    vector<double> trEDLT;
    vector<double> trEEPNP;
    vector<double> trEPOSIT;
    vector<double> trEOI;
    vector<double> trELM;

    double successDLT = 0;
    double successEPNP = 0;
    double successPOSIT = 0;
    double successLM = 0;
    double successOI = 0;

    for (unsigned nTr=0; nTr<uinTrials; nTr++)
    {
      // Generate random object
      object newObject;
      newObject.generateObject(25,25,25,nPt);

      // Generate random pose
      pose newPose;
      newPose.generatePose(0,0,0,0,0,40,PI,PI,PI,15,15,15);

      // Project to image
      image newImage;
      newImage.generateImage(newObject, newCamera, newPose);

      // Perturb Image Points
      newImage.perturbImage(generatorI, 0, 10);

      // DLT
      se3PoseDLT = normalizedDLT(newObject, newImage, newCamera);

      rotErrDLT = rotationError(newPose.getRotationMatrix(), se3PoseDLT.getRotationMatrix());
      trErrDLT = translationError(newPose.getTranstalion(), se3PoseDLT.getTranstalion());

      if (rotErrDLT < 181 && trErrDLT < 101)
      {
        rotEDLT.push_back(rotErrDLT);
        trEDLT.push_back(trErrDLT);
        successDLT += 1;
      }

      // EPNP
      se3PoseEPNP = EPNP(newObject, newImage, newCamera);

      rotErrEPNP = rotationError(newPose.getRotationMatrix(), se3PoseEPNP.getRotationMatrix());
      trErrEPNP = translationError(newPose.getTranstalion(), se3PoseEPNP.getTranstalion());

      if (rotErrEPNP < 181 && trErrEPNP < 101)
      {
        rotEEPNP.push_back(rotErrEPNP);
        trEEPNP.push_back(trErrEPNP);
        successEPNP += 1;
      }

      // POSIT
      se3PosePOSIT = POSIT(newObject, newImage, newCamera, 100);

      rotErrPOSIT = rotationError(newPose.getRotationMatrix(), se3PosePOSIT.getRotationMatrix());
      trErrPOSIT = translationError(newPose.getTranstalion(), se3PosePOSIT.getTranstalion());

      if (rotErrPOSIT < 181 && trErrPOSIT < 101)
      {
        rotEPOSIT.push_back(rotErrPOSIT);
        trEPOSIT.push_back(trErrPOSIT);
        successPOSIT += 1;
      }

      // OI
      se3PoseOI = OI(newObject, newImage, newCamera, se3initPose, 100);

      rotErrOI = rotationError(newPose.getRotationMatrix(), se3PoseOI.getRotationMatrix());
      trErrOI = translationError(newPose.getTranstalion(), se3PoseOI.getTranstalion());

      if (rotErrOI < 181 && trErrOI < 101)
      {
        rotEOI.push_back(rotErrOI);
        trEOI.push_back(trErrOI);
        successOI += 1;
      }

      // LM
      se3PoseLM = LM(newObject, newImage, newCamera, se3PoseEPNP, 100);

      rotErrLM = rotationError(newPose.getRotationMatrix(), se3PoseLM.getRotationMatrix());
      trErrLM = translationError(newPose.getTranstalion(), se3PoseLM.getTranstalion());

      if (rotErrLM < 181 && trErrLM < 101)
      {
        rotELM.push_back(rotErrLM);
        trELM.push_back(trErrLM);
        successLM += 1;
      }

      progressN = (double)nIteration/(double)tIterations*100;
      cout << "\r   Progress: " << progressN << " %      ";
      nIteration += 1;
    }

    vector<double> rotDLTQuartiles = findQuartiles(rotEDLT);
    rotErrDLTMean.push_back(findMean(rotEDLT));
    rotErrDLTMedian.push_back(rotDLTQuartiles[1]);
    rotErrDLTStdDev.push_back(findStdDev(rotEDLT));
    rotErrDLTFQuartile.push_back(rotDLTQuartiles[0]);
    rotErrDLTTQuartile.push_back(rotDLTQuartiles[2]);
    rotErrDLTMin.push_back(findMin2(rotEDLT));
    rotErrDLTMax.push_back(findMax2(rotEDLT));
    DLTSR.push_back(findnOutliers(rotEDLT, rotDLTQuartiles[0] - 1.5 * (rotDLTQuartiles[2] - rotDLTQuartiles[0]), rotDLTQuartiles[0] + 1.5 * (rotDLTQuartiles[2] - rotDLTQuartiles[0]))/uinTrials*100);

    vector<double> trDLTQuartiles = findQuartiles(trEDLT);
    trErrDLTMean.push_back(findMean(trEDLT));
    trErrDLTMedian.push_back(trDLTQuartiles[1]);
    trErrDLTStdDev.push_back(findStdDev(trEDLT));
    trErrDLTFQuartile.push_back(trDLTQuartiles[0]);
    trErrDLTTQuartile.push_back(trDLTQuartiles[2]);
    trErrDLTMin.push_back(findMin2(trEDLT));
    trErrDLTMax.push_back(findMax2(trEDLT));


    vector<double> rotEPNPQuartiles = findQuartiles(rotEEPNP);
    rotErrEPNPMean.push_back(findMean(rotEEPNP));
    rotErrEPNPMedian.push_back(rotEPNPQuartiles[1]);
    rotErrEPNPStdDev.push_back(findStdDev(rotEEPNP));
    rotErrEPNPFQuartile.push_back(rotEPNPQuartiles[0]);
    rotErrEPNPTQuartile.push_back(rotEPNPQuartiles[2]);
    rotErrEPNPMin.push_back(findMin2(rotEEPNP));
    rotErrEPNPMax.push_back(findMax2(rotEEPNP));
    EPNPSR.push_back(findnOutliers(rotEEPNP, rotEPNPQuartiles[0] - 1.5 * (rotEPNPQuartiles[2] - rotEPNPQuartiles[0]), rotEPNPQuartiles[0] + 1.5 * (rotEPNPQuartiles[2] - rotEPNPQuartiles[0]))/uinTrials*100);

    vector<double> trEPNPQuartiles = findQuartiles(trEEPNP);
    trErrEPNPMean.push_back(findMean(trEEPNP));
    trErrEPNPMedian.push_back(trEPNPQuartiles[1]);
    trErrEPNPStdDev.push_back(findStdDev(trEEPNP));
    trErrEPNPFQuartile.push_back(trEPNPQuartiles[0]);
    trErrEPNPTQuartile.push_back(trEPNPQuartiles[2]);
    trErrEPNPMin.push_back(findMin2(trEEPNP));
    trErrEPNPMax.push_back(findMax2(trEEPNP));


    vector<double> rotPOSITQuartiles = findQuartiles(rotEPOSIT);
    rotErrPOSITMean.push_back(findMean(rotEPOSIT));
    rotErrPOSITMedian.push_back(rotPOSITQuartiles[1]);
    rotErrPOSITStdDev.push_back(findStdDev(rotEPOSIT));
    rotErrPOSITFQuartile.push_back(rotPOSITQuartiles[0]);
    rotErrPOSITTQuartile.push_back(rotPOSITQuartiles[2]);
    rotErrPOSITMin.push_back(findMin2(rotEPOSIT));
    rotErrPOSITMax.push_back(findMax2(rotEPOSIT));
    POSITSR.push_back(findnOutliers(rotEPOSIT, rotPOSITQuartiles[0] - 1.5 * (rotPOSITQuartiles[2] - rotPOSITQuartiles[0]), rotPOSITQuartiles[0] + 1.5 * (rotPOSITQuartiles[2] - rotPOSITQuartiles[0]))/uinTrials*100);

    vector<double> trPOSITQuartiles = findQuartiles(trEPOSIT);
    trErrPOSITMean.push_back(findMean(trEPOSIT));
    trErrPOSITMedian.push_back(trPOSITQuartiles[1]);
    trErrPOSITStdDev.push_back(findStdDev(trEPOSIT));
    trErrPOSITFQuartile.push_back(trPOSITQuartiles[0]);
    trErrPOSITTQuartile.push_back(trPOSITQuartiles[2]);
    trErrPOSITMin.push_back(findMin2(trEPOSIT));
    trErrPOSITMax.push_back(findMax2(trEPOSIT));

    vector<double> rotOIQuartiles = findQuartiles(rotEOI);
    rotErrOIMean.push_back(findMean(rotEOI));
    rotErrOIMedian.push_back(rotOIQuartiles[1]);
    rotErrOIStdDev.push_back(findStdDev(rotEOI));
    rotErrOIFQuartile.push_back(rotOIQuartiles[0]);
    rotErrOITQuartile.push_back(rotOIQuartiles[2]);
    rotErrOIMin.push_back(findMin2(rotEOI));
    rotErrOIMax.push_back(findMax2(rotEOI));
    OISR.push_back(findnOutliers(rotEOI, rotOIQuartiles[0] - 1.5 * (rotOIQuartiles[2] - rotOIQuartiles[0]), rotOIQuartiles[0] + 1.5 * (rotOIQuartiles[2] - rotOIQuartiles[0]))/uinTrials*100);

    vector<double> trOIQuartiles = findQuartiles(trEOI);
    trErrOIMean.push_back(findMean(trEOI));
    trErrOIMedian.push_back(trOIQuartiles[1]);
    trErrOIStdDev.push_back(findStdDev(trEOI));
    trErrOIFQuartile.push_back(trOIQuartiles[0]);
    trErrOITQuartile.push_back(trOIQuartiles[2]);
    trErrOIMin.push_back(findMin2(trEOI));
    trErrOIMax.push_back(findMax2(trEOI));

    vector<double> rotLMQuartiles = findQuartiles(rotELM);
    rotErrLMMean.push_back(findMean(rotELM));
    rotErrLMMedian.push_back(rotLMQuartiles[1]);
    rotErrLMStdDev.push_back(findStdDev(rotELM));
    rotErrLMFQuartile.push_back(rotLMQuartiles[0]);
    rotErrLMTQuartile.push_back(rotLMQuartiles[2]);
    rotErrLMMin.push_back(findMin2(rotELM));
    rotErrLMMax.push_back(findMax2(rotELM));
    LMSR.push_back(findnOutliers(rotELM, rotLMQuartiles[0] - 1.5 * (rotLMQuartiles[2] - rotLMQuartiles[0]), rotLMQuartiles[0] + 1.5 * (rotLMQuartiles[2] - rotLMQuartiles[0]))/uinTrials*100);

    vector<double> trLMQuartiles = findQuartiles(trELM);
    trErrLMMean.push_back(findMean(trELM));
    trErrLMMedian.push_back(trLMQuartiles[1]);
    trErrLMStdDev.push_back(findStdDev(trELM));
    trErrLMFQuartile.push_back(trLMQuartiles[0]);
    trErrLMTQuartile.push_back(trLMQuartiles[2]);
    trErrLMMin.push_back(findMin2(trELM));
    trErrLMMax.push_back(findMax2(trELM));

  }

  ofstream fileA2a("plotA2a.dat");
  fileA2a << "# plotA2a.dat" << endl;
  fileA2a << "# Data file for GNUPLOT" << endl;
  fileA2a << "# Test A. No. of points" << endl;
  fileA2a << "# - A.2. Speed (nPts = 6-150, var = 5, outliers = 0 %)" << endl;
  fileA2a << "# - A.2.a. Rotation Error (degrees)" << endl;
  fileA2a << "# nPts  " << "\t"
         << "u-DLT   " << "\t" << "m-DLT   " << "\t" << "p-DLT   " << "\t" << "1q-DLT  " << "\t" << "3q-DLT  " << "\t" << "mn-DLT  " << "\t" << "mx-DLT  " << "\t" << "SR-DLT  " << "\t"
         << "u-EPN   " << "\t" << "m-EPN   " << "\t" << "p-EPN   " << "\t" << "1q-EPN  " << "\t" << "3q-EPN  " << "\t" << "mn-EPN  " << "\t" << "mx-EPN  " << "\t" << "SR-EPN  " << "\t"
         << "u-POS   " << "\t" << "m-POS   " << "\t" << "p-POS   " << "\t" << "1q-POS  " << "\t" << "3q-POS  " << "\t" << "mn-POS  " << "\t" << "mx-POS  " << "\t" << "SR-POS  " << "\t"
         << "u-OI    " << "\t" << "m-OI    " << "\t" << "p-OI    " << "\t" << "1q-OI   " << "\t" << "3q-OI   " << "\t" << "mn-OI   " << "\t" << "mx-OI   " << "\t" << "SR-OI   " << "\t"
         << "u-LM    " << "\t" << "m-LM    " << "\t" << "p-LM    " << "\t" << "1q-LM   " << "\t" << "3q-LM   " << "\t" << "mn-LM   " << "\t" << "mx-LM   " << "\t" << "SR-LM   " << "\t" << endl;

  for (unsigned i=0; i<=uinPtsU - uinPtsL; i++)
  {
    fileA2a << fixed << setprecision(6) << (double)i+uinPtsL << "\t"
           << rotErrDLTMean[i] << "\t" << rotErrDLTMedian[i] << "\t" << rotErrDLTStdDev[i] << "\t" << rotErrDLTFQuartile[i] << "\t" << rotErrDLTTQuartile[i] << "\t" << rotErrDLTMin[i] << "\t" << rotErrDLTMax[i] << "\t" << DLTSR[i] << "\t"
           << rotErrEPNPMean[i] << "\t" << rotErrEPNPMedian[i] << "\t" << rotErrEPNPStdDev[i] << "\t" << rotErrEPNPFQuartile[i] << "\t" << rotErrEPNPTQuartile[i] << "\t" << rotErrEPNPMin[i] << "\t" << rotErrEPNPMax[i] << "\t" << EPNPSR[i] << "\t"
           << rotErrPOSITMean[i] << "\t" << rotErrPOSITMedian[i] << "\t" << rotErrPOSITStdDev[i] << "\t" << rotErrPOSITFQuartile[i] << "\t" << rotErrPOSITTQuartile[i] << "\t" << rotErrPOSITMin[i] << "\t" << rotErrPOSITMax[i] << "\t" << POSITSR[i] << "\t"
           << rotErrOIMean[i] << "\t" << rotErrOIMedian[i] << "\t" << rotErrOIStdDev[i] << "\t" << rotErrOIFQuartile[i] << "\t" << rotErrOITQuartile[i] << "\t" << rotErrOIMin[i] << "\t" << rotErrOIMax[i] << "\t" << OISR[i] << "\t"
           << rotErrLMMean[i] << "\t" << rotErrLMMedian[i] << "\t" << rotErrLMStdDev[i] << "\t" << rotErrLMFQuartile[i] << "\t" << rotErrLMTQuartile[i] << "\t" << rotErrLMMin[i] << "\t" << rotErrLMMax[i] << "\t" << LMSR[i] << "\t" << endl;

  }

  fileA2a.close();

  ofstream fileA2b("plotA2b.dat");
  fileA2b << "# plotA2a.dat" << endl;
  fileA2b << "# Data file for GNUPLOT" << endl;
  fileA2b << "# Test A. No. of points" << endl;
  fileA2b << "# - A.2. Speed (nPts = 6-150, var = 5, outliers = 0 %)" << endl;
  fileA2b << "# - A.2.b. Translation Error (pct)" << endl;
  fileA2b << "# nPts  " << "\t"
         << "u-DLT   " << "\t" << "m-DLT   " << "\t" << "p-DLT   " << "\t" << "1q-DLT  " << "\t" << "3q-DLT  " << "\t" << "mn-DLT  " << "\t" << "mx-DLT  " << "\t" << "SR-DLT  " << "\t"
         << "u-EPN   " << "\t" << "m-EPN   " << "\t" << "p-EPN   " << "\t" << "1q-EPN  " << "\t" << "3q-EPN  " << "\t" << "mn-EPN  " << "\t" << "mx-EPN  " << "\t" << "SR-EPN  " << "\t"
         << "u-POS   " << "\t" << "m-POS   " << "\t" << "p-POS   " << "\t" << "1q-POS  " << "\t" << "3q-POS  " << "\t" << "mn-POS  " << "\t" << "mx-POS  " << "\t" << "SR-POS  " << "\t"
         << "u-OI    " << "\t" << "m-OI    " << "\t" << "p-OI    " << "\t" << "1q-OI   " << "\t" << "3q-OI   " << "\t" << "mn-OI   " << "\t" << "mx-OI   " << "\t" << "SR-OI   " << "\t"
         << "u-LM    " << "\t" << "m-LM    " << "\t" << "p-LM    " << "\t" << "1q-LM   " << "\t" << "3q-LM   " << "\t" << "mn-LM   " << "\t" << "mx-LM   " << "\t" << "SR-LM   " << "\t" << endl;

  for (unsigned i=0; i<=uinPtsU - uinPtsL; i++)
  {
    fileA2b << fixed << setprecision(6) << (double)i+uinPtsL << "\t"
           << trErrDLTMean[i] << "\t" << trErrDLTMedian[i] << "\t" << trErrDLTStdDev[i] << "\t" << trErrDLTFQuartile[i] << "\t" << trErrDLTTQuartile[i] << "\t" << trErrDLTMin[i] << "\t" << trErrDLTMax[i] << "\t" << DLTSR[i] << "\t"
           << trErrEPNPMean[i] << "\t" << trErrEPNPMedian[i] << "\t" << trErrEPNPStdDev[i] << "\t" << trErrEPNPFQuartile[i] << "\t" << trErrEPNPTQuartile[i] << "\t" << trErrEPNPMin[i] << "\t" << trErrEPNPMax[i] << "\t" << EPNPSR[i] << "\t"
           << trErrPOSITMean[i] << "\t" << trErrPOSITMedian[i] << "\t" << trErrPOSITStdDev[i] << "\t" << trErrPOSITFQuartile[i] << "\t" << trErrPOSITTQuartile[i] << "\t" << trErrPOSITMin[i] << "\t" << trErrPOSITMax[i] << "\t" << POSITSR[i] << "\t"
           << trErrOIMean[i] << "\t" << trErrOIMedian[i] << "\t" << trErrOIStdDev[i] << "\t" << trErrOIFQuartile[i] << "\t" << trErrOITQuartile[i] << "\t" << trErrOIMin[i] << "\t" << trErrOIMax[i] << "\t" << OISR[i] << "\t"
           << trErrLMMean[i] << "\t" << trErrLMMedian[i] << "\t" << trErrLMStdDev[i] << "\t" << trErrLMFQuartile[i] << "\t" << trErrLMTQuartile[i] << "\t" << trErrLMMin[i] << "\t" << trErrLMMax[i] << "\t" << LMSR[i] << "\t" << endl;

  }

  fileA2b.close();
  cout << endl << endl;

  unsigned noiseL = 0;
  unsigned noiseU = 15;
  uinTrials = 5000;

  tDLTMean.clear();
  tDLTMedian.clear();
  tDLTStdDev.clear();
  tDLTFQuartile.clear();
  tDLTTQuartile.clear();
  tDLTMin.clear();
  tDLTMax.clear();
  tDLTSR.clear();

  tEPNPMean.clear();
  tEPNPMedian.clear();
  tEPNPStdDev.clear();
  tEPNPFQuartile.clear();
  tEPNPTQuartile.clear();
  tEPNPMin.clear();
  tEPNPMax.clear();
  tEPNPSR.clear();

  tPOSITMean.clear();
  tPOSITMedian.clear();
  tPOSITStdDev.clear();
  tPOSITFQuartile.clear();
  tPOSITTQuartile.clear();
  tPOSITMin.clear();
  tPOSITMax.clear();
  tPOSITSR.clear();

  tOIMean.clear();
  tOIMedian.clear();
  tOIStdDev.clear();
  tOIFQuartile.clear();
  tOITQuartile.clear();
  tOIMin.clear();
  tOIMax.clear();
  tOISR.clear();

  tLMMean.clear();
  tLMMedian.clear();
  tLMStdDev.clear();
  tLMFQuartile.clear();
  tLMTQuartile.clear();
  tLMMin.clear();
  tLMMax.clear();
  tLMSR.clear();

  rotErrDLTMean.clear();
  rotErrDLTMedian.clear();
  rotErrDLTStdDev.clear();
  rotErrDLTFQuartile.clear();
  rotErrDLTTQuartile.clear();
  rotErrDLTMin.clear();
  rotErrDLTMax.clear();

  trErrDLTMean.clear();
  trErrDLTMedian.clear();
  trErrDLTStdDev.clear();
  trErrDLTFQuartile.clear();
  trErrDLTTQuartile.clear();
  trErrDLTMin.clear();
  trErrDLTMax.clear();

  DLTSR.clear();

  rotErrEPNPMean.clear();
  rotErrEPNPMedian.clear();
  rotErrEPNPStdDev.clear();
  rotErrEPNPFQuartile.clear();
  rotErrEPNPTQuartile.clear();
  rotErrEPNPMin.clear();
  rotErrEPNPMax.clear();

  trErrEPNPMean.clear();
  trErrEPNPMedian.clear();
  trErrEPNPStdDev.clear();
  trErrEPNPFQuartile.clear();
  trErrEPNPTQuartile.clear();
  trErrEPNPMin.clear();
  trErrEPNPMax.clear();

  EPNPSR.clear();

  rotErrPOSITMean.clear();
  rotErrPOSITMedian.clear();
  rotErrPOSITStdDev.clear();
  rotErrPOSITFQuartile.clear();
  rotErrPOSITTQuartile.clear();
  rotErrPOSITMin.clear();
  rotErrPOSITMax.clear();

  trErrPOSITMean.clear();
  trErrPOSITMedian.clear();
  trErrPOSITStdDev.clear();
  trErrPOSITFQuartile.clear();
  trErrPOSITTQuartile.clear();
  trErrPOSITMin.clear();
  trErrPOSITMax.clear();

  POSITSR.clear();

  rotErrOIMean.clear();
  rotErrOIMedian.clear();
  rotErrOIStdDev.clear();
  rotErrOIFQuartile.clear();
  rotErrOITQuartile.clear();
  rotErrOIMin.clear();
  rotErrOIMax.clear();

  trErrOIMean.clear();
  trErrOIMedian.clear();
  trErrOIStdDev.clear();
  trErrOIFQuartile.clear();
  trErrOITQuartile.clear();
  trErrOIMin.clear();
  trErrOIMax.clear();

  OISR.clear();

  rotErrLMMean.clear();
  rotErrLMMedian.clear();
  rotErrLMStdDev.clear();
  rotErrLMFQuartile.clear();
  rotErrLMTQuartile.clear();
  rotErrLMMin.clear();
  rotErrLMMax.clear();

  trErrLMMean.clear();
  trErrLMMedian.clear();
  trErrLMStdDev.clear();
  trErrLMFQuartile.clear();
  trErrLMTQuartile.clear();
  trErrLMMin.clear();
  trErrLMMax.clear();

  LMSR.clear();

  progressN = 0;
  progressL = 0;
  tIterations = (noiseU-noiseL+1)*uinTrials;
  nIteration = 0;

  cout << endl << "Running Test B. Noise" << endl << endl;

  for (unsigned nNoise=noiseL; nNoise <= noiseU-noiseL; nNoise++)
  {
    vector<double> tDLT;
    vector<double> tEPNP;
    vector<double> tPOSIT;
    vector<double> tOI;
    vector<double> tLM;

    vector<double> rotEDLT;
    vector<double> rotEEPNP;
    vector<double> rotEPOSIT;
    vector<double> rotEOI;
    vector<double> rotELM;

    vector<double> trEDLT;
    vector<double> trEEPNP;
    vector<double> trEPOSIT;
    vector<double> trEOI;
    vector<double> trELM;

    for (unsigned uiT=0; uiT<uinTrials; uiT++)
    {
      // Generate random object
      object newObject;
      newObject.generateObject(25,25,25,6);

      // Generate random pose
      pose newPose;
      newPose.generatePose(0,0,0,0,0,40,PI,PI,PI,15,15,15);

      // Project to image
      image newImage;
      newImage.generateImage(newObject, newCamera, newPose);

      // Perturb Image Points
      newImage.perturbImage(generatorI, 0, nNoise);

      // DLT
      gettimeofday(&t1, NULL);
      se3PoseDLT = normalizedDLT(newObject, newImage, newCamera);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrDLT = rotationError(newPose.getRotationMatrix(), se3PoseDLT.getRotationMatrix());
      trErrDLT = translationError(newPose.getTranstalion(), se3PoseDLT.getTranstalion());

      rotEDLT.push_back(rotErrDLT);
      trEDLT.push_back(trErrDLT);
      tDLT.push_back(elapsedTime);


      // EPNP
      gettimeofday(&t1, NULL);
      se3PoseEPNP = EPNP(newObject, newImage, newCamera);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrEPNP = rotationError(newPose.getRotationMatrix(), se3PoseEPNP.getRotationMatrix());
      trErrEPNP = translationError(newPose.getTranstalion(), se3PoseEPNP.getTranstalion());

      rotEEPNP.push_back(rotErrEPNP);
      trEEPNP.push_back(trErrEPNP);
      tEPNP.push_back(elapsedTime);

      // POSIT
      gettimeofday(&t1, NULL);
      se3PosePOSIT = POSIT(newObject, newImage, newCamera, 100);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrPOSIT = rotationError(newPose.getRotationMatrix(), se3PosePOSIT.getRotationMatrix());
      trErrPOSIT = translationError(newPose.getTranstalion(), se3PosePOSIT.getTranstalion());

      rotEPOSIT.push_back(rotErrPOSIT);
      trEPOSIT.push_back(trErrPOSIT);
      tPOSIT.push_back(elapsedTime);

      // OI
      gettimeofday(&t1, NULL);
      se3PoseOI = OI(newObject, newImage, newCamera, se3initPose, 100);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrOI = rotationError(newPose.getRotationMatrix(), se3PoseOI.getRotationMatrix());
      trErrOI = translationError(newPose.getTranstalion(), se3PoseOI.getTranstalion());

      rotEOI.push_back(rotErrOI);
      trEOI.push_back(trErrOI);
      tOI.push_back(elapsedTime);


      // LM
      gettimeofday(&t1, NULL);
      se3PoseLM = LM(newObject, newImage, newCamera, se3PoseEPNP, 100);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrLM = rotationError(newPose.getRotationMatrix(), se3PoseLM.getRotationMatrix());
      trErrLM = translationError(newPose.getTranstalion(), se3PoseLM.getTranstalion());

      rotELM.push_back(rotErrLM);
      trELM.push_back(trErrLM);
      tLM.push_back(elapsedTime);

      // Done estimating poses

      progressN = (double)nIteration/(double)tIterations*100;
      cout << "\r   Progress: " << progressN << " %      ";
      nIteration += 1;
    }

    vector<double> tDLTQuartiles = findQuartiles(tDLT);
    tDLTMean.push_back(findMean(tDLT));
    tDLTMedian.push_back(tDLTQuartiles[1]);
    tDLTStdDev.push_back(findStdDev(tDLT));
    tDLTFQuartile.push_back(tDLTQuartiles[0]);
    tDLTTQuartile.push_back(tDLTQuartiles[2]);
    tDLTMin.push_back(findMin2(tDLT));
    tDLTMax.push_back(findMax2(tDLT));
    tDLTSR.push_back(findnOutliers(tDLT, tDLTQuartiles[0] - 1.5 * (tDLTQuartiles[2] - tDLTQuartiles[0]), tDLTQuartiles[0] + 1.5 * (tDLTQuartiles[2] - tDLTQuartiles[0]))/uinTrials*100);

    vector<double> tEPNPQuartiles = findQuartiles(tEPNP);
    tEPNPMean.push_back(findMean(tEPNP));
    tEPNPMedian.push_back(tEPNPQuartiles[1]);
    tEPNPStdDev.push_back(findStdDev(tEPNP));
    tEPNPFQuartile.push_back(tEPNPQuartiles[0]);
    tEPNPTQuartile.push_back(tEPNPQuartiles[2]);
    tEPNPMin.push_back(findMin2(tEPNP));
    tEPNPMax.push_back(findMax2(tEPNP));
    tEPNPSR.push_back(findnOutliers(tEPNP, tEPNPQuartiles[0] - 1.5 * (tEPNPQuartiles[2] - tEPNPQuartiles[0]), tEPNPQuartiles[0] + 1.5 * (tEPNPQuartiles[2] - tEPNPQuartiles[0]))/uinTrials*100);

    vector<double> tPOSITQuartiles = findQuartiles(tPOSIT);
    tPOSITMean.push_back(findMean(tPOSIT));
    tPOSITMedian.push_back(tPOSITQuartiles[1]);
    tPOSITStdDev.push_back(findStdDev(tPOSIT));
    tPOSITFQuartile.push_back(tPOSITQuartiles[0]);
    tPOSITTQuartile.push_back(tPOSITQuartiles[2]);
    tPOSITMin.push_back(findMin2(tPOSIT));
    tPOSITMax.push_back(findMax2(tPOSIT));
    tPOSITSR.push_back(findnOutliers(tPOSIT, tPOSITQuartiles[0] - 1.5 * (tPOSITQuartiles[2] - tPOSITQuartiles[0]), tPOSITQuartiles[0] + 1.5 * (tPOSITQuartiles[2] - tPOSITQuartiles[0]))/uinTrials*100);

    vector<double> tOIQuartiles = findQuartiles(tOI);
    tOIMean.push_back(findMean(tOI));
    tOIMedian.push_back(tOIQuartiles[1]);
    tOIStdDev.push_back(findStdDev(tOI));
    tOIFQuartile.push_back(tOIQuartiles[0]);
    tOITQuartile.push_back(tOIQuartiles[2]);
    tOIMin.push_back(findMin2(tOI));
    tOIMax.push_back(findMax2(tOI));
    tOISR.push_back(findnOutliers(tOI, tOIQuartiles[0] - 1.5 * (tOIQuartiles[2] - tOIQuartiles[0]), tOIQuartiles[0] + 1.5 * (tOIQuartiles[2] - tOIQuartiles[0]))/uinTrials*100);

    vector<double> tLMQuartiles = findQuartiles(tLM);
    tLMMean.push_back(findMean(tLM));
    tLMMedian.push_back(tLMQuartiles[1]);
    tLMStdDev.push_back(findStdDev(tLM));
    tLMFQuartile.push_back(tLMQuartiles[0]);
    tLMTQuartile.push_back(tLMQuartiles[2]);
    tLMMin.push_back(findMin2(tLM));
    tLMMax.push_back(findMax2(tLM));
    tLMSR.push_back(findnOutliers(tLM, tLMQuartiles[0] - 1.5 * (tLMQuartiles[2] - tLMQuartiles[0]), tLMQuartiles[0] + 1.5 * (tLMQuartiles[2] - tLMQuartiles[0]))/uinTrials*100);

    vector<double> rotDLTQuartiles = findQuartiles(rotEDLT);
    rotErrDLTMean.push_back(findMean(rotEDLT));
    rotErrDLTMedian.push_back(rotDLTQuartiles[1]);
    rotErrDLTStdDev.push_back(findStdDev(rotEDLT));
    rotErrDLTFQuartile.push_back(rotDLTQuartiles[0]);
    rotErrDLTTQuartile.push_back(rotDLTQuartiles[2]);
    rotErrDLTMin.push_back(findMin2(rotEDLT));
    rotErrDLTMax.push_back(findMax2(rotEDLT));
    DLTSR.push_back(findnOutliers(rotEDLT, rotDLTQuartiles[0] - 1.5 * (rotDLTQuartiles[2] - rotDLTQuartiles[0]), rotDLTQuartiles[0] + 1.5 * (rotDLTQuartiles[2] - rotDLTQuartiles[0]))/uinTrials*100);

    vector<double> trDLTQuartiles = findQuartiles(trEDLT);
    trErrDLTMean.push_back(findMean(trEDLT));
    trErrDLTMedian.push_back(trDLTQuartiles[1]);
    trErrDLTStdDev.push_back(findStdDev(trEDLT));
    trErrDLTFQuartile.push_back(trDLTQuartiles[0]);
    trErrDLTTQuartile.push_back(trDLTQuartiles[2]);
    trErrDLTMin.push_back(findMin2(trEDLT));
    trErrDLTMax.push_back(findMax2(trEDLT));


    vector<double> rotEPNPQuartiles = findQuartiles(rotEEPNP);
    rotErrEPNPMean.push_back(findMean(rotEEPNP));
    rotErrEPNPMedian.push_back(rotEPNPQuartiles[1]);
    rotErrEPNPStdDev.push_back(findStdDev(rotEEPNP));
    rotErrEPNPFQuartile.push_back(rotEPNPQuartiles[0]);
    rotErrEPNPTQuartile.push_back(rotEPNPQuartiles[2]);
    rotErrEPNPMin.push_back(findMin2(rotEEPNP));
    rotErrEPNPMax.push_back(findMax2(rotEEPNP));
    EPNPSR.push_back(findnOutliers(rotEEPNP, rotEPNPQuartiles[0] - 1.5 * (rotEPNPQuartiles[2] - rotEPNPQuartiles[0]), rotEPNPQuartiles[0] + 1.5 * (rotEPNPQuartiles[2] - rotEPNPQuartiles[0]))/uinTrials*100);

    vector<double> trEPNPQuartiles = findQuartiles(trEEPNP);
    trErrEPNPMean.push_back(findMean(trEEPNP));
    trErrEPNPMedian.push_back(trEPNPQuartiles[1]);
    trErrEPNPStdDev.push_back(findStdDev(trEEPNP));
    trErrEPNPFQuartile.push_back(trEPNPQuartiles[0]);
    trErrEPNPTQuartile.push_back(trEPNPQuartiles[2]);
    trErrEPNPMin.push_back(findMin2(trEEPNP));
    trErrEPNPMax.push_back(findMax2(trEEPNP));


    vector<double> rotPOSITQuartiles = findQuartiles(rotEPOSIT);
    rotErrPOSITMean.push_back(findMean(rotEPOSIT));
    rotErrPOSITMedian.push_back(rotPOSITQuartiles[1]);
    rotErrPOSITStdDev.push_back(findStdDev(rotEPOSIT));
    rotErrPOSITFQuartile.push_back(rotPOSITQuartiles[0]);
    rotErrPOSITTQuartile.push_back(rotPOSITQuartiles[2]);
    rotErrPOSITMin.push_back(findMin2(rotEPOSIT));
    rotErrPOSITMax.push_back(findMax2(rotEPOSIT));
    POSITSR.push_back(findnOutliers(rotEPOSIT, rotPOSITQuartiles[0] - 1.5 * (rotPOSITQuartiles[2] - rotPOSITQuartiles[0]), rotPOSITQuartiles[0] + 1.5 * (rotPOSITQuartiles[2] - rotPOSITQuartiles[0]))/uinTrials*100);

    vector<double> trPOSITQuartiles = findQuartiles(trEPOSIT);
    trErrPOSITMean.push_back(findMean(trEPOSIT));
    trErrPOSITMedian.push_back(trPOSITQuartiles[1]);
    trErrPOSITStdDev.push_back(findStdDev(trEPOSIT));
    trErrPOSITFQuartile.push_back(trPOSITQuartiles[0]);
    trErrPOSITTQuartile.push_back(trPOSITQuartiles[2]);
    trErrPOSITMin.push_back(findMin2(trEPOSIT));
    trErrPOSITMax.push_back(findMax2(trEPOSIT));

    vector<double> rotOIQuartiles = findQuartiles(rotEOI);
    rotErrOIMean.push_back(findMean(rotEOI));
    rotErrOIMedian.push_back(rotOIQuartiles[1]);
    rotErrOIStdDev.push_back(findStdDev(rotEOI));
    rotErrOIFQuartile.push_back(rotOIQuartiles[0]);
    rotErrOITQuartile.push_back(rotOIQuartiles[2]);
    rotErrOIMin.push_back(findMin2(rotEOI));
    rotErrOIMax.push_back(findMax2(rotEOI));
    OISR.push_back(findnOutliers(rotEOI, rotOIQuartiles[0] - 1.5 * (rotOIQuartiles[2] - rotOIQuartiles[0]), rotOIQuartiles[0] + 1.5 * (rotOIQuartiles[2] - rotOIQuartiles[0]))/uinTrials*100);

    vector<double> trOIQuartiles = findQuartiles(trEOI);
    trErrOIMean.push_back(findMean(trEOI));
    trErrOIMedian.push_back(trOIQuartiles[1]);
    trErrOIStdDev.push_back(findStdDev(trEOI));
    trErrOIFQuartile.push_back(trOIQuartiles[0]);
    trErrOITQuartile.push_back(trOIQuartiles[2]);
    trErrOIMin.push_back(findMin2(trEOI));
    trErrOIMax.push_back(findMax2(trEOI));

    vector<double> rotLMQuartiles = findQuartiles(rotELM);
    rotErrLMMean.push_back(findMean(rotELM));
    rotErrLMMedian.push_back(rotLMQuartiles[1]);
    rotErrLMStdDev.push_back(findStdDev(rotELM));
    rotErrLMFQuartile.push_back(rotLMQuartiles[0]);
    rotErrLMTQuartile.push_back(rotLMQuartiles[2]);
    rotErrLMMin.push_back(findMin2(rotELM));
    rotErrLMMax.push_back(findMax2(rotELM));
    LMSR.push_back(findnOutliers(rotELM, rotLMQuartiles[0] - 1.5 * (rotLMQuartiles[2] - rotLMQuartiles[0]), rotLMQuartiles[0] + 1.5 * (rotLMQuartiles[2] - rotLMQuartiles[0]))/uinTrials*100);

    vector<double> trLMQuartiles = findQuartiles(trELM);
    trErrLMMean.push_back(findMean(trELM));
    trErrLMMedian.push_back(trLMQuartiles[1]);
    trErrLMStdDev.push_back(findStdDev(trELM));
    trErrLMFQuartile.push_back(trLMQuartiles[0]);
    trErrLMTQuartile.push_back(trLMQuartiles[2]);
    trErrLMMin.push_back(findMin2(trELM));
    trErrLMMax.push_back(findMax2(trELM));
  }

  ofstream fileB1("plotB1.dat");
  fileB1 << "# plotB1.dat" << endl;
  fileB1 << "# Data file for GNUPLOT" << endl;
  fileB1 << "# Test B. Noise" << endl;
  fileB1 << "# - B.1. Speed" << endl;
  fileB1 << "# nNoi  " << "\t"
         << "u-DLT   " << "\t" << "m-DLT   " << "\t" << "p-DLT   " << "\t" << "1q-DLT  " << "\t" << "3q-DLT  " << "\t" << "mn-DLT  " << "\t" << "mx-DLT  " << "\t" << "SR-DLT  " << "\t"
         << "u-EPN   " << "\t" << "m-EPN   " << "\t" << "p-EPN   " << "\t" << "1q-EPN  " << "\t" << "3q-EPN  " << "\t" << "mn-EPN  " << "\t" << "mx-EPN  " << "\t" << "SR-EPN  " << "\t"
         << "u-POS   " << "\t" << "m-POS   " << "\t" << "p-POS   " << "\t" << "1q-POS  " << "\t" << "3q-POS  " << "\t" << "mn-POS  " << "\t" << "mx-POS  " << "\t" << "SR-POS  " << "\t"
         << "u-OI    " << "\t" << "m-OI    " << "\t" << "p-OI    " << "\t" << "1q-OI   " << "\t" << "3q-OI   " << "\t" << "mn-OI   " << "\t" << "mx-OI   " << "\t" << "SR-OI   " << "\t"
         << "u-LM    " << "\t" << "m-LM    " << "\t" << "p-LM    " << "\t" << "1q-LM   " << "\t" << "3q-LM   " << "\t" << "mn-LM   " << "\t" << "mx-LM   " << "\t" << "SR-LM   " << "\t" << endl;

  for (unsigned i=noiseL; i<=noiseU - noiseL; i++)
  {
    fileB1 << fixed << setprecision(6) << (double)i << "\t"
           << tDLTMean[i] << "\t" << tDLTMedian[i] << "\t" << tDLTStdDev[i] << "\t" << tDLTFQuartile[i] << "\t" << tDLTTQuartile[i] << "\t" << tDLTMin[i] << "\t" << tDLTMax[i] << "\t" << tDLTSR[i] << "\t"
           << tEPNPMean[i] << "\t" << tEPNPMedian[i] << "\t" << tEPNPStdDev[i] << "\t" << tEPNPFQuartile[i] << "\t" << tEPNPTQuartile[i] << "\t" << tEPNPMin[i] << "\t" << tEPNPMax[i] << "\t" << tEPNPSR[i] << "\t"
           << tPOSITMean[i] << "\t" << tPOSITMedian[i] << "\t" << tPOSITStdDev[i] << "\t" << tPOSITFQuartile[i] << "\t" << tPOSITTQuartile[i] << "\t" << tPOSITMin[i] << "\t" << tPOSITMax[i] << "\t" << tPOSITSR[i] << "\t"
           << tOIMean[i] << "\t" << tOIMedian[i] << "\t" << tOIStdDev[i] << "\t" << tOIFQuartile[i] << "\t" << tOITQuartile[i] << "\t" << tOIMin[i] << "\t" << tOIMax[i] << "\t" << tOISR[i] << "\t"
           << tLMMean[i] << "\t" << tLMMedian[i] << "\t" << tLMStdDev[i] << "\t" << tLMFQuartile[i] << "\t" << tLMTQuartile[i] << "\t" << tLMMin[i] << "\t" << tLMMax[i] << "\t" << tLMSR[i] << "\t" << endl;

  }

  fileB1.close();

  ofstream fileB2a("plotB2a.dat");
  fileB2a << "# plotB2a.dat" << endl;
  fileB2a << "# Data file for GNUPLOT" << endl;
  fileB2a << "# Test B. Noise" << endl;
  fileB2a << "# - B.2. Accuracy" << endl;
  fileB2a << "# - B.2.a. Rotation Error (degrees)" << endl;
  fileB2a << "# nNoi  " << "\t"
         << "u-DLT   " << "\t" << "m-DLT   " << "\t" << "p-DLT   " << "\t" << "1q-DLT  " << "\t" << "3q-DLT  " << "\t" << "mn-DLT  " << "\t" << "mx-DLT  " << "\t" << "SR-DLT  " << "\t"
         << "u-EPN   " << "\t" << "m-EPN   " << "\t" << "p-EPN   " << "\t" << "1q-EPN  " << "\t" << "3q-EPN  " << "\t" << "mn-EPN  " << "\t" << "mx-EPN  " << "\t" << "SR-EPN  " << "\t"
         << "u-POS   " << "\t" << "m-POS   " << "\t" << "p-POS   " << "\t" << "1q-POS  " << "\t" << "3q-POS  " << "\t" << "mn-POS  " << "\t" << "mx-POS  " << "\t" << "SR-POS  " << "\t"
         << "u-OI    " << "\t" << "m-OI    " << "\t" << "p-OI    " << "\t" << "1q-OI   " << "\t" << "3q-OI   " << "\t" << "mn-OI   " << "\t" << "mx-OI   " << "\t" << "SR-OI   " << "\t"
         << "u-LM    " << "\t" << "m-LM    " << "\t" << "p-LM    " << "\t" << "1q-LM   " << "\t" << "3q-LM   " << "\t" << "mn-LM   " << "\t" << "mx-LM   " << "\t" << "SR-LM   " << "\t" << endl;

  for (unsigned i=noiseL; i<=noiseU - noiseL; i++)
  {
    fileB2a << fixed << setprecision(6) << (double)i << "\t"
           << rotErrDLTMean[i] << "\t" << rotErrDLTMedian[i] << "\t" << rotErrDLTStdDev[i] << "\t" << rotErrDLTFQuartile[i] << "\t" << rotErrDLTTQuartile[i] << "\t" << rotErrDLTMin[i] << "\t" << rotErrDLTMax[i] << "\t" << DLTSR[i] << "\t"
           << rotErrEPNPMean[i] << "\t" << rotErrEPNPMedian[i] << "\t" << rotErrEPNPStdDev[i] << "\t" << rotErrEPNPFQuartile[i] << "\t" << rotErrEPNPTQuartile[i] << "\t" << rotErrEPNPMin[i] << "\t" << rotErrEPNPMax[i] << "\t" << EPNPSR[i] << "\t"
           << rotErrPOSITMean[i] << "\t" << rotErrPOSITMedian[i] << "\t" << rotErrPOSITStdDev[i] << "\t" << rotErrPOSITFQuartile[i] << "\t" << rotErrPOSITTQuartile[i] << "\t" << rotErrPOSITMin[i] << "\t" << rotErrPOSITMax[i] << "\t" << POSITSR[i] << "\t"
           << rotErrOIMean[i] << "\t" << rotErrOIMedian[i] << "\t" << rotErrOIStdDev[i] << "\t" << rotErrOIFQuartile[i] << "\t" << rotErrOITQuartile[i] << "\t" << rotErrOIMin[i] << "\t" << rotErrOIMax[i] << "\t" << OISR[i] << "\t"
           << rotErrLMMean[i] << "\t" << rotErrLMMedian[i] << "\t" << rotErrLMStdDev[i] << "\t" << rotErrLMFQuartile[i] << "\t" << rotErrLMTQuartile[i] << "\t" << rotErrLMMin[i] << "\t" << rotErrLMMax[i] << "\t" << LMSR[i] << "\t" << endl;

  }

  fileB2a.close();

  ofstream fileB2b("plotB2b.dat");
  fileB2b << "# plotB2b.dat" << endl;
  fileB2b << "# Data file for GNUPLOT" << endl;
  fileB2b << "# Test B. Noise" << endl;
  fileB2b << "# - B.2. Accuracy" << endl;
  fileB2b << "# - B.2.b. Translation Error" << endl;
  fileB2b << "# nPts  " << "\t"
         << "u-DLT   " << "\t" << "m-DLT   " << "\t" << "p-DLT   " << "\t" << "1q-DLT  " << "\t" << "3q-DLT  " << "\t" << "mn-DLT  " << "\t" << "mx-DLT  " << "\t" << "SR-DLT  " << "\t"
         << "u-EPN   " << "\t" << "m-EPN   " << "\t" << "p-EPN   " << "\t" << "1q-EPN  " << "\t" << "3q-EPN  " << "\t" << "mn-EPN  " << "\t" << "mx-EPN  " << "\t" << "SR-EPN  " << "\t"
         << "u-POS   " << "\t" << "m-POS   " << "\t" << "p-POS   " << "\t" << "1q-POS  " << "\t" << "3q-POS  " << "\t" << "mn-POS  " << "\t" << "mx-POS  " << "\t" << "SR-POS  " << "\t"
         << "u-OI    " << "\t" << "m-OI    " << "\t" << "p-OI    " << "\t" << "1q-OI   " << "\t" << "3q-OI   " << "\t" << "mn-OI   " << "\t" << "mx-OI   " << "\t" << "SR-OI   " << "\t"
         << "u-LM    " << "\t" << "m-LM    " << "\t" << "p-LM    " << "\t" << "1q-LM   " << "\t" << "3q-LM   " << "\t" << "mn-LM   " << "\t" << "mx-LM   " << "\t" << "SR-LM   " << "\t" << endl;

  for (unsigned i=noiseL; i<=noiseU - noiseL; i++)
  {
    fileB2b << fixed << setprecision(6) << (double)i << "\t"
           << trErrDLTMean[i] << "\t" << trErrDLTMedian[i] << "\t" << trErrDLTStdDev[i] << "\t" << trErrDLTFQuartile[i] << "\t" << trErrDLTTQuartile[i] << "\t" << trErrDLTMin[i] << "\t" << trErrDLTMax[i] << "\t" << DLTSR[i] << "\t"
           << trErrEPNPMean[i] << "\t" << trErrEPNPMedian[i] << "\t" << trErrEPNPStdDev[i] << "\t" << trErrEPNPFQuartile[i] << "\t" << trErrEPNPTQuartile[i] << "\t" << trErrEPNPMin[i] << "\t" << trErrEPNPMax[i] << "\t" << EPNPSR[i] << "\t"
           << trErrPOSITMean[i] << "\t" << trErrPOSITMedian[i] << "\t" << trErrPOSITStdDev[i] << "\t" << trErrPOSITFQuartile[i] << "\t" << trErrPOSITTQuartile[i] << "\t" << trErrPOSITMin[i] << "\t" << trErrPOSITMax[i] << "\t" << POSITSR[i] << "\t"
           << trErrOIMean[i] << "\t" << trErrOIMedian[i] << "\t" << trErrOIStdDev[i] << "\t" << trErrOIFQuartile[i] << "\t" << trErrOITQuartile[i] << "\t" << trErrOIMin[i] << "\t" << trErrOIMax[i] << "\t" << OISR[i] << "\t"
           << trErrLMMean[i] << "\t" << trErrLMMedian[i] << "\t" << trErrLMStdDev[i] << "\t" << trErrLMFQuartile[i] << "\t" << trErrLMTQuartile[i] << "\t" << trErrLMMin[i] << "\t" << trErrLMMax[i] << "\t" << LMSR[i] << "\t" << endl;

  }

  fileB2b.close();
  cout << endl << endl;

  unsigned distU = 50;
  unsigned distL = 0;
  uinTrials = 1000;

  tDLTMean.clear();
  tDLTMedian.clear();
  tDLTStdDev.clear();
  tDLTFQuartile.clear();
  tDLTTQuartile.clear();
  tDLTMin.clear();
  tDLTMax.clear();
  tDLTSR.clear();

  tEPNPMean.clear();
  tEPNPMedian.clear();
  tEPNPStdDev.clear();
  tEPNPFQuartile.clear();
  tEPNPTQuartile.clear();
  tEPNPMin.clear();
  tEPNPMax.clear();
  tEPNPSR.clear();

  tPOSITMean.clear();
  tPOSITMedian.clear();
  tPOSITStdDev.clear();
  tPOSITFQuartile.clear();
  tPOSITTQuartile.clear();
  tPOSITMin.clear();
  tPOSITMax.clear();
  tPOSITSR.clear();

  tOIMean.clear();
  tOIMedian.clear();
  tOIStdDev.clear();
  tOIFQuartile.clear();
  tOITQuartile.clear();
  tOIMin.clear();
  tOIMax.clear();
  tOISR.clear();

  tLMMean.clear();
  tLMMedian.clear();
  tLMStdDev.clear();
  tLMFQuartile.clear();
  tLMTQuartile.clear();
  tLMMin.clear();
  tLMMax.clear();
  tLMSR.clear();

  rotErrDLTMean.clear();
  rotErrDLTMedian.clear();
  rotErrDLTStdDev.clear();
  rotErrDLTFQuartile.clear();
  rotErrDLTTQuartile.clear();
  rotErrDLTMin.clear();
  rotErrDLTMax.clear();

  trErrDLTMean.clear();
  trErrDLTMedian.clear();
  trErrDLTStdDev.clear();
  trErrDLTFQuartile.clear();
  trErrDLTTQuartile.clear();
  trErrDLTMin.clear();
  trErrDLTMax.clear();

  DLTSR.clear();

  rotErrEPNPMean.clear();
  rotErrEPNPMedian.clear();
  rotErrEPNPStdDev.clear();
  rotErrEPNPFQuartile.clear();
  rotErrEPNPTQuartile.clear();
  rotErrEPNPMin.clear();
  rotErrEPNPMax.clear();

  trErrEPNPMean.clear();
  trErrEPNPMedian.clear();
  trErrEPNPStdDev.clear();
  trErrEPNPFQuartile.clear();
  trErrEPNPTQuartile.clear();
  trErrEPNPMin.clear();
  trErrEPNPMax.clear();

  EPNPSR.clear();

  rotErrPOSITMean.clear();
  rotErrPOSITMedian.clear();
  rotErrPOSITStdDev.clear();
  rotErrPOSITFQuartile.clear();
  rotErrPOSITTQuartile.clear();
  rotErrPOSITMin.clear();
  rotErrPOSITMax.clear();

  trErrPOSITMean.clear();
  trErrPOSITMedian.clear();
  trErrPOSITStdDev.clear();
  trErrPOSITFQuartile.clear();
  trErrPOSITTQuartile.clear();
  trErrPOSITMin.clear();
  trErrPOSITMax.clear();

  POSITSR.clear();

  rotErrOIMean.clear();
  rotErrOIMedian.clear();
  rotErrOIStdDev.clear();
  rotErrOIFQuartile.clear();
  rotErrOITQuartile.clear();
  rotErrOIMin.clear();
  rotErrOIMax.clear();

  trErrOIMean.clear();
  trErrOIMedian.clear();
  trErrOIStdDev.clear();
  trErrOIFQuartile.clear();
  trErrOITQuartile.clear();
  trErrOIMin.clear();
  trErrOIMax.clear();

  OISR.clear();

  rotErrLMMean.clear();
  rotErrLMMedian.clear();
  rotErrLMStdDev.clear();
  rotErrLMFQuartile.clear();
  rotErrLMTQuartile.clear();
  rotErrLMMin.clear();
  rotErrLMMax.clear();

  trErrLMMean.clear();
  trErrLMMedian.clear();
  trErrLMStdDev.clear();
  trErrLMFQuartile.clear();
  trErrLMTQuartile.clear();
  trErrLMMin.clear();
  trErrLMMax.clear();

  LMSR.clear();

  progressN = 0;
  progressL = 0;
  tIterations = (distU-distL+1)*uinTrials;
  nIteration = 0;

  cout << endl << "Running Test C. Distance from the Camera" << endl << endl;

  for (unsigned nDist=distL; nDist <= distU-distL; nDist++)
  {
    vector<double> tDLT;
    vector<double> tEPNP;
    vector<double> tPOSIT;
    vector<double> tOI;
    vector<double> tLM;

    vector<double> rotEDLT;
    vector<double> rotEEPNP;
    vector<double> rotEPOSIT;
    vector<double> rotEOI;
    vector<double> rotELM;

    vector<double> trEDLT;
    vector<double> trEEPNP;
    vector<double> trEPOSIT;
    vector<double> trEOI;
    vector<double> trELM;

    for (unsigned uiT=0; uiT<uinTrials; uiT++)
    {
      // Generate random object
      object newObject;
      newObject.generateObject(25,25,25,6);

      // Generate random pose
      pose newPose;
      newPose.generatePose(0,0,0,0,0,26+nDist,PI,PI,PI,15,15,0);

      // Project to image
      image newImage;
      newImage.generateImage(newObject, newCamera, newPose);

      // Perturb Image Points
      newImage.perturbImage(generatorI, 0, 2);

      // DLT
      gettimeofday(&t1, NULL);
      se3PoseDLT = normalizedDLT(newObject, newImage, newCamera);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrDLT = rotationError(newPose.getRotationMatrix(), se3PoseDLT.getRotationMatrix());
      trErrDLT = translationError(newPose.getTranstalion(), se3PoseDLT.getTranstalion());

      rotEDLT.push_back(rotErrDLT);
      trEDLT.push_back(trErrDLT);
      tDLT.push_back(elapsedTime);


      // EPNP
      gettimeofday(&t1, NULL);
      se3PoseEPNP = EPNP(newObject, newImage, newCamera);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrEPNP = rotationError(newPose.getRotationMatrix(), se3PoseEPNP.getRotationMatrix());
      trErrEPNP = translationError(newPose.getTranstalion(), se3PoseEPNP.getTranstalion());

      rotEEPNP.push_back(rotErrEPNP);
      trEEPNP.push_back(trErrEPNP);
      tEPNP.push_back(elapsedTime);

      // POSIT
      gettimeofday(&t1, NULL);
      se3PosePOSIT = POSIT(newObject, newImage, newCamera, 100);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrPOSIT = rotationError(newPose.getRotationMatrix(), se3PosePOSIT.getRotationMatrix());
      trErrPOSIT = translationError(newPose.getTranstalion(), se3PosePOSIT.getTranstalion());

      rotEPOSIT.push_back(rotErrPOSIT);
      trEPOSIT.push_back(trErrPOSIT);
      tPOSIT.push_back(elapsedTime);

      // OI
      gettimeofday(&t1, NULL);
      se3PoseOI = OI(newObject, newImage, newCamera, se3initPose, 100);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrOI = rotationError(newPose.getRotationMatrix(), se3PoseOI.getRotationMatrix());
      trErrOI = translationError(newPose.getTranstalion(), se3PoseOI.getTranstalion());

      rotEOI.push_back(rotErrOI);
      trEOI.push_back(trErrOI);
      tOI.push_back(elapsedTime);


      // LM
      gettimeofday(&t1, NULL);
      se3PoseLM = LM(newObject, newImage, newCamera, se3PoseEPNP, 100);
      gettimeofday(&t2, NULL);

      elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
      elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;

      rotErrLM = rotationError(newPose.getRotationMatrix(), se3PoseLM.getRotationMatrix());
      trErrLM = translationError(newPose.getTranstalion(), se3PoseLM.getTranstalion());

      rotELM.push_back(rotErrLM);
      trELM.push_back(trErrLM);
      tLM.push_back(elapsedTime);

      // Done estimating poses

      progressN = (double)nIteration/(double)tIterations*100;
      cout << "\r   Progress: " << progressN << " %      ";
      nIteration += 1;
    }

    vector<double> tDLTQuartiles = findQuartiles(tDLT);
    tDLTMean.push_back(findMean(tDLT));
    tDLTMedian.push_back(tDLTQuartiles[1]);
    tDLTStdDev.push_back(findStdDev(tDLT));
    tDLTFQuartile.push_back(tDLTQuartiles[0]);
    tDLTTQuartile.push_back(tDLTQuartiles[2]);
    tDLTMin.push_back(findMin2(tDLT));
    tDLTMax.push_back(findMax2(tDLT));
    tDLTSR.push_back(findnOutliers(tDLT, tDLTQuartiles[0] - 1.5 * (tDLTQuartiles[2] - tDLTQuartiles[0]), tDLTQuartiles[0] + 1.5 * (tDLTQuartiles[2] - tDLTQuartiles[0]))/uinTrials*100);

    vector<double> tEPNPQuartiles = findQuartiles(tEPNP);
    tEPNPMean.push_back(findMean(tEPNP));
    tEPNPMedian.push_back(tEPNPQuartiles[1]);
    tEPNPStdDev.push_back(findStdDev(tEPNP));
    tEPNPFQuartile.push_back(tEPNPQuartiles[0]);
    tEPNPTQuartile.push_back(tEPNPQuartiles[2]);
    tEPNPMin.push_back(findMin2(tEPNP));
    tEPNPMax.push_back(findMax2(tEPNP));
    tEPNPSR.push_back(findnOutliers(tEPNP, tEPNPQuartiles[0] - 1.5 * (tEPNPQuartiles[2] - tEPNPQuartiles[0]), tEPNPQuartiles[0] + 1.5 * (tEPNPQuartiles[2] - tEPNPQuartiles[0]))/uinTrials*100);

    vector<double> tPOSITQuartiles = findQuartiles(tPOSIT);
    tPOSITMean.push_back(findMean(tPOSIT));
    tPOSITMedian.push_back(tPOSITQuartiles[1]);
    tPOSITStdDev.push_back(findStdDev(tPOSIT));
    tPOSITFQuartile.push_back(tPOSITQuartiles[0]);
    tPOSITTQuartile.push_back(tPOSITQuartiles[2]);
    tPOSITMin.push_back(findMin2(tPOSIT));
    tPOSITMax.push_back(findMax2(tPOSIT));
    tPOSITSR.push_back(findnOutliers(tPOSIT, tPOSITQuartiles[0] - 1.5 * (tPOSITQuartiles[2] - tPOSITQuartiles[0]), tPOSITQuartiles[0] + 1.5 * (tPOSITQuartiles[2] - tPOSITQuartiles[0]))/uinTrials*100);

    vector<double> tOIQuartiles = findQuartiles(tOI);
    tOIMean.push_back(findMean(tOI));
    tOIMedian.push_back(tOIQuartiles[1]);
    tOIStdDev.push_back(findStdDev(tOI));
    tOIFQuartile.push_back(tOIQuartiles[0]);
    tOITQuartile.push_back(tOIQuartiles[2]);
    tOIMin.push_back(findMin2(tOI));
    tOIMax.push_back(findMax2(tOI));
    tOISR.push_back(findnOutliers(tOI, tOIQuartiles[0] - 1.5 * (tOIQuartiles[2] - tOIQuartiles[0]), tOIQuartiles[0] + 1.5 * (tOIQuartiles[2] - tOIQuartiles[0]))/uinTrials*100);

    vector<double> tLMQuartiles = findQuartiles(tLM);
    tLMMean.push_back(findMean(tLM));
    tLMMedian.push_back(tLMQuartiles[1]);
    tLMStdDev.push_back(findStdDev(tLM));
    tLMFQuartile.push_back(tLMQuartiles[0]);
    tLMTQuartile.push_back(tLMQuartiles[2]);
    tLMMin.push_back(findMin2(tLM));
    tLMMax.push_back(findMax2(tLM));
    tLMSR.push_back(findnOutliers(tLM, tLMQuartiles[0] - 1.5 * (tLMQuartiles[2] - tLMQuartiles[0]), tLMQuartiles[0] + 1.5 * (tLMQuartiles[2] - tLMQuartiles[0]))/uinTrials*100);

    vector<double> rotDLTQuartiles = findQuartiles(rotEDLT);
    rotErrDLTMean.push_back(findMean(rotEDLT));
    rotErrDLTMedian.push_back(rotDLTQuartiles[1]);
    rotErrDLTStdDev.push_back(findStdDev(rotEDLT));
    rotErrDLTFQuartile.push_back(rotDLTQuartiles[0]);
    rotErrDLTTQuartile.push_back(rotDLTQuartiles[2]);
    rotErrDLTMin.push_back(findMin2(rotEDLT));
    rotErrDLTMax.push_back(findMax2(rotEDLT));
    DLTSR.push_back(findnOutliers(rotEDLT, rotDLTQuartiles[0] - 1.5 * (rotDLTQuartiles[2] - rotDLTQuartiles[0]), rotDLTQuartiles[0] + 1.5 * (rotDLTQuartiles[2] - rotDLTQuartiles[0]))/uinTrials*100);

    vector<double> trDLTQuartiles = findQuartiles(trEDLT);
    trErrDLTMean.push_back(findMean(trEDLT));
    trErrDLTMedian.push_back(trDLTQuartiles[1]);
    trErrDLTStdDev.push_back(findStdDev(trEDLT));
    trErrDLTFQuartile.push_back(trDLTQuartiles[0]);
    trErrDLTTQuartile.push_back(trDLTQuartiles[2]);
    trErrDLTMin.push_back(findMin2(trEDLT));
    trErrDLTMax.push_back(findMax2(trEDLT));


    vector<double> rotEPNPQuartiles = findQuartiles(rotEEPNP);
    rotErrEPNPMean.push_back(findMean(rotEEPNP));
    rotErrEPNPMedian.push_back(rotEPNPQuartiles[1]);
    rotErrEPNPStdDev.push_back(findStdDev(rotEEPNP));
    rotErrEPNPFQuartile.push_back(rotEPNPQuartiles[0]);
    rotErrEPNPTQuartile.push_back(rotEPNPQuartiles[2]);
    rotErrEPNPMin.push_back(findMin2(rotEEPNP));
    rotErrEPNPMax.push_back(findMax2(rotEEPNP));
    EPNPSR.push_back(findnOutliers(rotEEPNP, rotEPNPQuartiles[0] - 1.5 * (rotEPNPQuartiles[2] - rotEPNPQuartiles[0]), rotEPNPQuartiles[0] + 1.5 * (rotEPNPQuartiles[2] - rotEPNPQuartiles[0]))/uinTrials*100);

    vector<double> trEPNPQuartiles = findQuartiles(trEEPNP);
    trErrEPNPMean.push_back(findMean(trEEPNP));
    trErrEPNPMedian.push_back(trEPNPQuartiles[1]);
    trErrEPNPStdDev.push_back(findStdDev(trEEPNP));
    trErrEPNPFQuartile.push_back(trEPNPQuartiles[0]);
    trErrEPNPTQuartile.push_back(trEPNPQuartiles[2]);
    trErrEPNPMin.push_back(findMin2(trEEPNP));
    trErrEPNPMax.push_back(findMax2(trEEPNP));


    vector<double> rotPOSITQuartiles = findQuartiles(rotEPOSIT);
    rotErrPOSITMean.push_back(findMean(rotEPOSIT));
    rotErrPOSITMedian.push_back(rotPOSITQuartiles[1]);
    rotErrPOSITStdDev.push_back(findStdDev(rotEPOSIT));
    rotErrPOSITFQuartile.push_back(rotPOSITQuartiles[0]);
    rotErrPOSITTQuartile.push_back(rotPOSITQuartiles[2]);
    rotErrPOSITMin.push_back(findMin2(rotEPOSIT));
    rotErrPOSITMax.push_back(findMax2(rotEPOSIT));
    POSITSR.push_back(findnOutliers(rotEPOSIT, rotPOSITQuartiles[0] - 1.5 * (rotPOSITQuartiles[2] - rotPOSITQuartiles[0]), rotPOSITQuartiles[0] + 1.5 * (rotPOSITQuartiles[2] - rotPOSITQuartiles[0]))/uinTrials*100);

    vector<double> trPOSITQuartiles = findQuartiles(trEPOSIT);
    trErrPOSITMean.push_back(findMean(trEPOSIT));
    trErrPOSITMedian.push_back(trPOSITQuartiles[1]);
    trErrPOSITStdDev.push_back(findStdDev(trEPOSIT));
    trErrPOSITFQuartile.push_back(trPOSITQuartiles[0]);
    trErrPOSITTQuartile.push_back(trPOSITQuartiles[2]);
    trErrPOSITMin.push_back(findMin2(trEPOSIT));
    trErrPOSITMax.push_back(findMax2(trEPOSIT));

    vector<double> rotOIQuartiles = findQuartiles(rotEOI);
    rotErrOIMean.push_back(findMean(rotEOI));
    rotErrOIMedian.push_back(rotOIQuartiles[1]);
    rotErrOIStdDev.push_back(findStdDev(rotEOI));
    rotErrOIFQuartile.push_back(rotOIQuartiles[0]);
    rotErrOITQuartile.push_back(rotOIQuartiles[2]);
    rotErrOIMin.push_back(findMin2(rotEOI));
    rotErrOIMax.push_back(findMax2(rotEOI));
    OISR.push_back(findnOutliers(rotEOI, rotOIQuartiles[0] - 1.5 * (rotOIQuartiles[2] - rotOIQuartiles[0]), rotOIQuartiles[0] + 1.5 * (rotOIQuartiles[2] - rotOIQuartiles[0]))/uinTrials*100);

    vector<double> trOIQuartiles = findQuartiles(trEOI);
    trErrOIMean.push_back(findMean(trEOI));
    trErrOIMedian.push_back(trOIQuartiles[1]);
    trErrOIStdDev.push_back(findStdDev(trEOI));
    trErrOIFQuartile.push_back(trOIQuartiles[0]);
    trErrOITQuartile.push_back(trOIQuartiles[2]);
    trErrOIMin.push_back(findMin2(trEOI));
    trErrOIMax.push_back(findMax2(trEOI));

    vector<double> rotLMQuartiles = findQuartiles(rotELM);
    rotErrLMMean.push_back(findMean(rotELM));
    rotErrLMMedian.push_back(rotLMQuartiles[1]);
    rotErrLMStdDev.push_back(findStdDev(rotELM));
    rotErrLMFQuartile.push_back(rotLMQuartiles[0]);
    rotErrLMTQuartile.push_back(rotLMQuartiles[2]);
    rotErrLMMin.push_back(findMin2(rotELM));
    rotErrLMMax.push_back(findMax2(rotELM));
    LMSR.push_back(findnOutliers(rotELM, rotLMQuartiles[0] - 1.5 * (rotLMQuartiles[2] - rotLMQuartiles[0]), rotLMQuartiles[0] + 1.5 * (rotLMQuartiles[2] - rotLMQuartiles[0]))/uinTrials*100);

    vector<double> trLMQuartiles = findQuartiles(trELM);
    trErrLMMean.push_back(findMean(trELM));
    trErrLMMedian.push_back(trLMQuartiles[1]);
    trErrLMStdDev.push_back(findStdDev(trELM));
    trErrLMFQuartile.push_back(trLMQuartiles[0]);
    trErrLMTQuartile.push_back(trLMQuartiles[2]);
    trErrLMMin.push_back(findMin2(trELM));
    trErrLMMax.push_back(findMax2(trELM));
  }

  ofstream fileC1("plotC1.dat");
  fileC1 << "# plotC1.dat" << endl;
  fileC1 << "# Data file for GNUPLOT" << endl;
  fileC1 << "# Test C. Distance from the camera" << endl;
  fileC1 << "# - C.1. Speed" << endl;
  fileC1 << "# nDst  " << "\t"
         << "u-DLT   " << "\t" << "m-DLT   " << "\t" << "p-DLT   " << "\t" << "1q-DLT  " << "\t" << "3q-DLT  " << "\t" << "mn-DLT  " << "\t" << "mx-DLT  " << "\t" << "SR-DLT  " << "\t"
         << "u-EPN   " << "\t" << "m-EPN   " << "\t" << "p-EPN   " << "\t" << "1q-EPN  " << "\t" << "3q-EPN  " << "\t" << "mn-EPN  " << "\t" << "mx-EPN  " << "\t" << "SR-EPN  " << "\t"
         << "u-POS   " << "\t" << "m-POS   " << "\t" << "p-POS   " << "\t" << "1q-POS  " << "\t" << "3q-POS  " << "\t" << "mn-POS  " << "\t" << "mx-POS  " << "\t" << "SR-POS  " << "\t"
         << "u-OI    " << "\t" << "m-OI    " << "\t" << "p-OI    " << "\t" << "1q-OI   " << "\t" << "3q-OI   " << "\t" << "mn-OI   " << "\t" << "mx-OI   " << "\t" << "SR-OI   " << "\t"
         << "u-LM    " << "\t" << "m-LM    " << "\t" << "p-LM    " << "\t" << "1q-LM   " << "\t" << "3q-LM   " << "\t" << "mn-LM   " << "\t" << "mx-LM   " << "\t" << "SR-LM   " << "\t" << endl;

  for (unsigned i=distL; i<=distU-distL; i++)
  {
    fileC1 << fixed << setprecision(6) << (double)i << "\t"
           << tDLTMean[i] << "\t" << tDLTMedian[i] << "\t" << tDLTStdDev[i] << "\t" << tDLTFQuartile[i] << "\t" << tDLTTQuartile[i] << "\t" << tDLTMin[i] << "\t" << tDLTMax[i] << "\t" << tDLTSR[i] << "\t"
           << tEPNPMean[i] << "\t" << tEPNPMedian[i] << "\t" << tEPNPStdDev[i] << "\t" << tEPNPFQuartile[i] << "\t" << tEPNPTQuartile[i] << "\t" << tEPNPMin[i] << "\t" << tEPNPMax[i] << "\t" << tEPNPSR[i] << "\t"
           << tPOSITMean[i] << "\t" << tPOSITMedian[i] << "\t" << tPOSITStdDev[i] << "\t" << tPOSITFQuartile[i] << "\t" << tPOSITTQuartile[i] << "\t" << tPOSITMin[i] << "\t" << tPOSITMax[i] << "\t" << tPOSITSR[i] << "\t"
           << tOIMean[i] << "\t" << tOIMedian[i] << "\t" << tOIStdDev[i] << "\t" << tOIFQuartile[i] << "\t" << tOITQuartile[i] << "\t" << tOIMin[i] << "\t" << tOIMax[i] << "\t" << tOISR[i] << "\t"
           << tLMMean[i] << "\t" << tLMMedian[i] << "\t" << tLMStdDev[i] << "\t" << tLMFQuartile[i] << "\t" << tLMTQuartile[i] << "\t" << tLMMin[i] << "\t" << tLMMax[i] << "\t" << tLMSR[i] << "\t" << endl;

  }

  fileC1.close();

  ofstream fileC2a("plotC2a.dat");
  fileC2a << "# plotC2a.dat" << endl;
  fileC2a << "# Data file for GNUPLOT" << endl;
  fileC2a << "# Test C. Distance from the camera" << endl;
  fileC2a << "# - C.2. Accuracy" << endl;
  fileC2a << "# - C.2.a. Rotation Error (degrees)" << endl;
  fileC2a << "# nDst  " << "\t"
         << "u-DLT   " << "\t" << "m-DLT   " << "\t" << "p-DLT   " << "\t" << "1q-DLT  " << "\t" << "3q-DLT  " << "\t" << "mn-DLT  " << "\t" << "mx-DLT  " << "\t" << "SR-DLT  " << "\t"
         << "u-EPN   " << "\t" << "m-EPN   " << "\t" << "p-EPN   " << "\t" << "1q-EPN  " << "\t" << "3q-EPN  " << "\t" << "mn-EPN  " << "\t" << "mx-EPN  " << "\t" << "SR-EPN  " << "\t"
         << "u-POS   " << "\t" << "m-POS   " << "\t" << "p-POS   " << "\t" << "1q-POS  " << "\t" << "3q-POS  " << "\t" << "mn-POS  " << "\t" << "mx-POS  " << "\t" << "SR-POS  " << "\t"
         << "u-OI    " << "\t" << "m-OI    " << "\t" << "p-OI    " << "\t" << "1q-OI   " << "\t" << "3q-OI   " << "\t" << "mn-OI   " << "\t" << "mx-OI   " << "\t" << "SR-OI   " << "\t"
         << "u-LM    " << "\t" << "m-LM    " << "\t" << "p-LM    " << "\t" << "1q-LM   " << "\t" << "3q-LM   " << "\t" << "mn-LM   " << "\t" << "mx-LM   " << "\t" << "SR-LM   " << "\t" << endl;

  for (unsigned i=distL; i<=distU-distL; i++)
  {
    fileC2a << fixed << setprecision(6) << (double)i << "\t"
           << rotErrDLTMean[i] << "\t" << rotErrDLTMedian[i] << "\t" << rotErrDLTStdDev[i] << "\t" << rotErrDLTFQuartile[i] << "\t" << rotErrDLTTQuartile[i] << "\t" << rotErrDLTMin[i] << "\t" << rotErrDLTMax[i] << "\t" << DLTSR[i] << "\t"
           << rotErrEPNPMean[i] << "\t" << rotErrEPNPMedian[i] << "\t" << rotErrEPNPStdDev[i] << "\t" << rotErrEPNPFQuartile[i] << "\t" << rotErrEPNPTQuartile[i] << "\t" << rotErrEPNPMin[i] << "\t" << rotErrEPNPMax[i] << "\t" << EPNPSR[i] << "\t"
           << rotErrPOSITMean[i] << "\t" << rotErrPOSITMedian[i] << "\t" << rotErrPOSITStdDev[i] << "\t" << rotErrPOSITFQuartile[i] << "\t" << rotErrPOSITTQuartile[i] << "\t" << rotErrPOSITMin[i] << "\t" << rotErrPOSITMax[i] << "\t" << POSITSR[i] << "\t"
           << rotErrOIMean[i] << "\t" << rotErrOIMedian[i] << "\t" << rotErrOIStdDev[i] << "\t" << rotErrOIFQuartile[i] << "\t" << rotErrOITQuartile[i] << "\t" << rotErrOIMin[i] << "\t" << rotErrOIMax[i] << "\t" << OISR[i] << "\t"
           << rotErrLMMean[i] << "\t" << rotErrLMMedian[i] << "\t" << rotErrLMStdDev[i] << "\t" << rotErrLMFQuartile[i] << "\t" << rotErrLMTQuartile[i] << "\t" << rotErrLMMin[i] << "\t" << rotErrLMMax[i] << "\t" << LMSR[i] << "\t" << endl;

  }

  fileC2a.close();

  ofstream fileC2b("plotC2b.dat");
  fileC2b << "# plotC2b.dat" << endl;
  fileC2b << "# Data file for GNUPLOT" << endl;
  fileC2b << "# Test C. Distance from the camera" << endl;
  fileC2b << "# - C.2. Accuracy" << endl;
  fileC2b << "# - C.2.b. Translation Error" << endl;
  fileC2b << "# nDst  " << "\t"
         << "u-DLT   " << "\t" << "m-DLT   " << "\t" << "p-DLT   " << "\t" << "1q-DLT  " << "\t" << "3q-DLT  " << "\t" << "mn-DLT  " << "\t" << "mx-DLT  " << "\t" << "SR-DLT  " << "\t"
         << "u-EPN   " << "\t" << "m-EPN   " << "\t" << "p-EPN   " << "\t" << "1q-EPN  " << "\t" << "3q-EPN  " << "\t" << "mn-EPN  " << "\t" << "mx-EPN  " << "\t" << "SR-EPN  " << "\t"
         << "u-POS   " << "\t" << "m-POS   " << "\t" << "p-POS   " << "\t" << "1q-POS  " << "\t" << "3q-POS  " << "\t" << "mn-POS  " << "\t" << "mx-POS  " << "\t" << "SR-POS  " << "\t"
         << "u-OI    " << "\t" << "m-OI    " << "\t" << "p-OI    " << "\t" << "1q-OI   " << "\t" << "3q-OI   " << "\t" << "mn-OI   " << "\t" << "mx-OI   " << "\t" << "SR-OI   " << "\t"
         << "u-LM    " << "\t" << "m-LM    " << "\t" << "p-LM    " << "\t" << "1q-LM   " << "\t" << "3q-LM   " << "\t" << "mn-LM   " << "\t" << "mx-LM   " << "\t" << "SR-LM   " << "\t" << endl;

  for (unsigned i=distL; i<=distU-distL; i++)
  {
    fileC2b << fixed << setprecision(6) << (double)i << "\t"
           << trErrDLTMean[i] << "\t" << trErrDLTMedian[i] << "\t" << trErrDLTStdDev[i] << "\t" << trErrDLTFQuartile[i] << "\t" << trErrDLTTQuartile[i] << "\t" << trErrDLTMin[i] << "\t" << trErrDLTMax[i] << "\t" << DLTSR[i] << "\t"
           << trErrEPNPMean[i] << "\t" << trErrEPNPMedian[i] << "\t" << trErrEPNPStdDev[i] << "\t" << trErrEPNPFQuartile[i] << "\t" << trErrEPNPTQuartile[i] << "\t" << trErrEPNPMin[i] << "\t" << trErrEPNPMax[i] << "\t" << EPNPSR[i] << "\t"
           << trErrPOSITMean[i] << "\t" << trErrPOSITMedian[i] << "\t" << trErrPOSITStdDev[i] << "\t" << trErrPOSITFQuartile[i] << "\t" << trErrPOSITTQuartile[i] << "\t" << trErrPOSITMin[i] << "\t" << trErrPOSITMax[i] << "\t" << POSITSR[i] << "\t"
           << trErrOIMean[i] << "\t" << trErrOIMedian[i] << "\t" << trErrOIStdDev[i] << "\t" << trErrOIFQuartile[i] << "\t" << trErrOITQuartile[i] << "\t" << trErrOIMin[i] << "\t" << trErrOIMax[i] << "\t" << OISR[i] << "\t"
           << trErrLMMean[i] << "\t" << trErrLMMedian[i] << "\t" << trErrLMStdDev[i] << "\t" << trErrLMFQuartile[i] << "\t" << trErrLMTQuartile[i] << "\t" << trErrLMMin[i] << "\t" << trErrLMMax[i] << "\t" << LMSR[i] << "\t" << endl;

  }

  fileC2b.close();
  cout << endl << endl;
}
