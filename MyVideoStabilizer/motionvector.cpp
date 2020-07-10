#include "motionvector.h"

MotionVector::MotionVector()
{
    this->x = 0;
    this->y = 0;
    this->startPoint = cv::Point2f(0, 0);
    this->endPoint = cv::Point2f(0, 0);
    this->magnitude = 0;
    this->angle = 0;
}

MotionVector::MotionVector(cv::Point2f startPoint, cv::Point2f endPoint)
{
    this->x = endPoint.x - startPoint.x;
    this->y = endPoint.y - startPoint.y;
    this->startPoint = startPoint;
    this->endPoint = endPoint;
    this->calculateMagnitude();
    this->calculateAngle();
}

MotionVector::MotionVector(const MotionVector& motionVector)
{
    this->x = motionVector.x;
    this->y = motionVector.y;
    this->startPoint = motionVector.startPoint;
    this->endPoint = motionVector.endPoint;
    this->magnitude = motionVector.magnitude;
    this->angle = motionVector.angle;
}


double MotionVector::calculateMagnitude(MotionVector motionVector)
{
    return sqrt(pow(motionVector.x, 2) + pow(motionVector.y, 2));
}

double MotionVector::calculateAngle(MotionVector motionVector)
{
    return atan2(motionVector.y, motionVector.x);
}

void MotionVector::calculateMagnitude()
{
    this->magnitude = sqrt(pow(this->x, 2) + pow(this->y, 2));
}

void MotionVector::calculateAngle()
{
    this->angle = atan2(this->y, this->x);
}

bool MotionVector::magnitudesComparer(MotionVector firstMotionVector, MotionVector secondMotionVector)
{
    return (firstMotionVector.magnitude < secondMotionVector.magnitude);
}

bool MotionVector::anglesComparer(MotionVector firstMotionVector, MotionVector secondMotionVector)
{
    return (firstMotionVector.angle < secondMotionVector.angle);
}
