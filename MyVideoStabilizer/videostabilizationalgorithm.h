#ifndef VIDEOSTABILIZATIONALGORITHM_H
#define VIDEOSTABILIZATIONALGORITHM_H

#include "motionvector.h"
#include "measurement.h"
#include "constants.h"


class VideoStabilizationAlgorithm
{
public:
    VideoStabilizationAlgorithm();

    // implementation of outliers elimination algorithm
    // double ended queue is used for optimization
    std::vector<MotionVector> OutliersElimination(const std::vector<MotionVector>& matchedKeypoints, unsigned int NumberOfOutliersToRemove);

    void filterKeypoints(const vector<MotionVector>& matchedKeypoints, vector<Point2f>& correspondingKeyPointsOfFirstFrame,
                         vector<Point2f>& correspondingKeyPointsOfSecondFrame);

    // function which removes keypoints on edges of image,
    // so that every keypoint contains pixels in every direction within window radius
    void removeKeyPointsOnBordersOfImage(std::vector<Point2f>& keyPointsOfFrame, const Mat& image);

    // function assumes that every keypoint contains pixels in every direction within window radius
    double calculateDifferenceBetweenKeyPoints(const Mat& firstGrayFrame, const Mat& secondGrayFrame,
                                               const Point2f& firstKeyPoint, const Point2f& secondKeyPoint);

    void findBestMatchesBetweenTwoFrames(const Mat& firstGrayFrame, const Mat& secondGrayFrame,
                                         const vector<Point2f>& keyPointsOfFirstFrame,
                                         const vector<Point2f>& keyPointsOfSecondFrame,
                                         vector<MotionVector>& matchedKeypoints);

    void resetTransformParameters(float transformParameters[4]);

    void smoothTransformParameters(float transformParameters[4]);

    void truncateTransformParameters(float transformParameters[4]);

    void extractAffineTransformParametersFromAffineMatrix(const Mat_<float>& affineTransformMatrix,
                                                          float transformParameters[4]);

    Mat_<float> createAffineTransformFromTransformParameters(float transformParameters[4]);

    void paintKeyPointsOnFrame(const Mat& originalFrame, const vector<Point2f>& correspondingKeyPointsOfFirstFrame,
                               const vector<Point2f>& correspondingKeyPointsOfSecondFrame);

    bool isTransformInvalid(const Mat_<float> &transformMatrix);

    bool isTransformErroneous(float transformParameters[4]);

    bool isTransformExaggerated(float transformParameters[4]);

    bool isPixelBlack(const Mat &frame, int x, int y);

    void removeEntirelyBlackPixelsFromImage(Mat& image);

    void truncateFrameToSmallerRectangle(Mat& frame, Rect2i newFrameRectangle, const Mat_<float> &transformMatrix);

    cv::Rect2i findBiggestNonBlackRectangleWithinFrame(const Mat &frame);

    cv::Rect2i findIntersectingRectangle(Rect2i rectangleA, Rect2i rectangleB);

    void processAlgorithm();
};

#endif // VIDEOSTABILIZATIONALGORITHM_H
