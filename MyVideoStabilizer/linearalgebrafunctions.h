#ifndef LINEARALGEBRAFUNCTIONS_H
#define LINEARALGEBRAFUNCTIONS_H

#include <opencv2/core/core.hpp>

double calculateVectorMagnitude(cv::Point2f vector);
double calculateVectorAngle(cv::Point2f vector);

cv::Point2f convertTwoPointsToVector(cv::Point2f firstPoint, cv::Point2f secondPoint);
cv::Point2f convertTwoPointsToVector(std::pair<cv::Point2f, cv::Point2f> pairOfPoints);

bool vectorMagnitudesComparer(std::pair<cv::Point2f, cv::Point2f> firstPairOfPoints,
                              std::pair<cv::Point2f, cv::Point2f> secondPairOfPoints);

bool vectorAnglesComparer(std::pair<cv::Point2f, cv::Point2f> firstPairOfPoints,
                          std::pair<cv::Point2f, cv::Point2f> secondPairOfPoints);

#endif // LINEARALGEBRAFUNCTIONS_H
