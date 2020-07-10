#include "linearalgebrafunctions.h"

double calculateVectorMagnitude(cv::Point2f vector)
{
    return sqrt(pow(vector.x, 2) + pow(vector.y, 2));
}

double calculateVectorAngle(cv::Point2f vector)
{
    return atan2(vector.y, vector.x);
}

cv::Point2f convertTwoPointsToVector(cv::Point2f firstPoint, cv::Point2f secondPoint)
{
    return cv::Point2f(secondPoint.x - firstPoint.x, secondPoint.y - firstPoint.y);
}

cv::Point2f convertTwoPointsToVector(std::pair<cv::Point2f, cv::Point2f> pairOfPoints)
{
    return cv::Point2f(pairOfPoints.second.x - pairOfPoints.first.x, pairOfPoints.second.y - pairOfPoints.first.y);
}

bool vectorMagnitudesComparer(std::pair<cv::Point2f, cv::Point2f> firstPairOfPoints,
                              std::pair<cv::Point2f, cv::Point2f> secondPairOfPoints)
{
    cv::Point2f firstMovementVector = convertTwoPointsToVector(firstPairOfPoints);
    cv::Point2f secondMovementVector = convertTwoPointsToVector(secondPairOfPoints);

    double firstVectorMagnitude = calculateVectorMagnitude(firstMovementVector);
    double secondVectorMagnitude = calculateVectorMagnitude(secondMovementVector);

    return (firstVectorMagnitude < secondVectorMagnitude);
}

bool vectorAnglesComparer(std::pair<cv::Point2f, cv::Point2f> firstPairOfPoints,
                          std::pair<cv::Point2f, cv::Point2f> secondPairOfPoints)
{
    cv::Point2f firstMovementVector = convertTwoPointsToVector(firstPairOfPoints);
    cv::Point2f secondMovementVector = convertTwoPointsToVector(secondPairOfPoints);

    double firstVectorAngle = calculateVectorAngle(firstMovementVector);
    double secondVectorAngle = calculateVectorAngle(secondMovementVector);

    return (firstVectorAngle < secondVectorAngle);
}
