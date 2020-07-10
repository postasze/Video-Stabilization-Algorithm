#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <fstream>      // std::ofstream
#include <vector>
#include <deque>
#include <algorithm>   // sort
#include <cmath>       // sqrt, atan, atan2
#include <iomanip>     // setprecision

using namespace cv;
using namespace std;

const int NUMBER_OF_KEYPOINTS = 300;
const int MINIMUM_NUMBER_OF_KEYPOINTS_TO_FIND = 40;
const float OUTLIERS_DISCARD_PERCENTAGE = 0.95f;
const float ALPHA = 0.95f;
const float QUALITY_LEVEL = 0.015f;
const int MINIMUM_DISTANCE = 12;
const int WINDOW_RADIUS = 10;

const float ACCEPTABLE_X_TRANSLATION_THRESHOLD = 45.0f;
const float ACCEPTABLE_Y_TRANSLATION_THRESHOLD = 30.0f;
const float ACCEPTABLE_SCALE_THRESHOLD = 0.1f;
const float ACCEPTABLE_ROTATION_THRESHOLD = 0.05f;

const float ERRONEOUS_X_TRANSLATION_THRESHOLD = 70.0f;
const float ERRONEOUS_Y_TRANSLATION_THRESHOLD = 50.0f;
const float ERRONEOUS_SCALE_THRESHOLD = 0.2f;
const float ERRONEOUS_ROTATION_THRESHOLD = 0.2f;

#endif // CONSTANTS_H
