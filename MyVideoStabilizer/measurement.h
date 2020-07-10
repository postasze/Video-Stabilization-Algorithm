#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <vector>

using namespace cv;

class Measurement
{
public:
    Measurement();

    void startMeasurement();
    void turnMeasurementOn();
    void turnMeasurementOff();
    void finishMeasurement();
    void measureStabilizationQuality(const Mat_<float>& warpingTransform, const Mat& grayScaleFrame);
    float measureCroppingRatio(const Mat_<float>& warpingTransform, const Mat& grayScaleFrame);
    float measureDistortionValue(const Mat_<float>& warpingTransform);
    float measureStabilityScore(const std::vector<float>& measuredRotations,
        const std::vector<float>& measuredXTranslations, const std::vector<float>& measuredYTranslations);
    void removeEntirelyBlackPixelsFromGrayScaleImage(Mat& grayScaleImage);
    void extractAffineTransformParametersFromAffineMatrix(const Mat_<float> &affineTransformMatrix,
                                                          float& xTranslation, float& yTranslation,
                                                          float& scale, float& theta);


    bool isMeasurementStarted;
    bool isMeasurementOn;
    bool isMeasurementFinished;
    float croppingRatio;
    float distortionValue;
    float stabilityScore;
    Mat_<float> cumulativeWarpingTransform;
    std::vector<float> measuredRotations, measuredXTranslations, measuredYTranslations;
    int measurementNumber;
    float measuredCroppingRatioSum;
    float xTranslationStabilityScore;
    float yTranslationStabilityScore;
    float rotationStabilityScore;
};

#endif // MEASUREMENT_H
