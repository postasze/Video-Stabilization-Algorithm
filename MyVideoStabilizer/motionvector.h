#ifndef MOTIONVECTOR_H
#define MOTIONVECTOR_H

#include <opencv2/core/core.hpp>

class MotionVector
{
public:
    MotionVector();
    MotionVector(cv::Point2f startPoint, cv::Point2f endPoint);
    MotionVector(const MotionVector& motionVector);

    int x, y; // calculated from difference between endPoint and startPoint
    cv::Point2f startPoint;
    cv::Point2f endPoint;
    double magnitude; // from 0 to infinity
    double angle; // from -pi to pi

    static double calculateMagnitude(MotionVector motionVector);
    static double calculateAngle(MotionVector motionVector);

    void calculateMagnitude();
    void calculateAngle();

    static bool magnitudesComparer(MotionVector firstMotionVector, MotionVector secondMotionVector);
    static bool anglesComparer(MotionVector firstMotionVector, MotionVector secondMotionVector);
};

#endif // MOTIONVECTOR_H
