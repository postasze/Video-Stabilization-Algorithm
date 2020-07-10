#include "measurement.h"
#include <iostream>

Measurement::Measurement()
{
    this->croppingRatio = 1;
    this->distortionValue = 1;
    this->stabilityScore = 1;
    this->isMeasurementStarted = false;
    this->isMeasurementOn = false;
    this->isMeasurementFinished = false;
    this->cumulativeWarpingTransform = Mat::eye(3,3,CV_32FC1);
    this->measurementNumber = 0;
    this->measuredCroppingRatioSum = 0;
    this->xTranslationStabilityScore = 1.0;
    this->yTranslationStabilityScore = 1.0;
    this->rotationStabilityScore = 1.0;
    measuredRotations.clear();
    measuredXTranslations.clear();
    measuredYTranslations.clear();
}

void Measurement::startMeasurement()
{
    this->croppingRatio = 1;
    this->distortionValue = 1;
    this->stabilityScore = 1;
    this->isMeasurementStarted = true;
    this->isMeasurementOn = true;
    this->isMeasurementFinished = false;
    this->cumulativeWarpingTransform = Mat::eye(3,3,CV_32FC1);
    this->measurementNumber = 0;
    this->measuredCroppingRatioSum = 0;
    this->xTranslationStabilityScore = 1.0;
    this->yTranslationStabilityScore = 1.0;
    this->rotationStabilityScore = 1.0;
    measuredRotations.clear();
    measuredXTranslations.clear();
    measuredYTranslations.clear();
}

void Measurement::turnMeasurementOn()
{
    if(this->isMeasurementStarted && !this->isMeasurementFinished)
        this->isMeasurementOn = true;
}

void Measurement::turnMeasurementOff()
{
    if(this->isMeasurementStarted && !this->isMeasurementFinished)
        this->isMeasurementOn = false;
}

void Measurement::finishMeasurement()
{
    if(!this->isMeasurementStarted)
        return;

    this->isMeasurementFinished = true;
    this->isMeasurementOn = false;

    this->croppingRatio = this->measuredCroppingRatioSum / this->measurementNumber;

    this->stabilityScore = measureStabilityScore(this->measuredRotations, this->measuredXTranslations, this->measuredYTranslations);
}

void Measurement::measureStabilizationQuality(const Mat_<float>& warpingTransform, const Mat& grayScaleFrame)
{
    float xTranslation, yTranslation, scale, theta;
    float currentlyMeasuredCroppingRatio, currentlyMeasuredDistortionValue;
    Mat copiedGrayScaleFrame = grayScaleFrame.clone();

    if(!this->isMeasurementStarted || !isMeasurementOn || isMeasurementFinished)
        return;

    measurementNumber++;

    removeEntirelyBlackPixelsFromGrayScaleImage(copiedGrayScaleFrame);
    extractAffineTransformParametersFromAffineMatrix(warpingTransform, xTranslation, yTranslation, scale, theta);
    cumulativeWarpingTransform *= warpingTransform;
    measuredRotations.push_back(theta);
    measuredXTranslations.push_back(xTranslation);
    measuredYTranslations.push_back(yTranslation);

    currentlyMeasuredCroppingRatio = measureCroppingRatio(warpingTransform, copiedGrayScaleFrame);
    currentlyMeasuredDistortionValue = measureDistortionValue(warpingTransform);

    if(currentlyMeasuredDistortionValue < this->distortionValue)
        this->distortionValue = currentlyMeasuredDistortionValue;

    this->measuredCroppingRatioSum += currentlyMeasuredCroppingRatio;
}

float Measurement::measureCroppingRatio(const Mat_<float>& warpingTransform, const Mat& grayScaleFrame)
{
    Mat grayScaleFrameWarped;
    int numberOfCroppedOutPixels = 0, numberOfAllFramePixels = grayScaleFrame.rows * grayScaleFrame.cols;

    warpAffine(grayScaleFrame, grayScaleFrameWarped, warpingTransform.rowRange(0, 2), Size());

    for(int i = 0; i < grayScaleFrame.rows; i++)
        for(int j = 0; j < grayScaleFrame.cols; j++)
            if(grayScaleFrameWarped.at<char>(i, j) == 0)
                numberOfCroppedOutPixels++;

    return (float)(numberOfAllFramePixels - numberOfCroppedOutPixels) / (float)numberOfAllFramePixels;
}

float Measurement::measureDistortionValue(const Mat_<float>& warpingTransform)
{
    Mat_<float> affinePart = Mat::eye(2,2,CV_32FC1);
    Mat_<float> eigenValues;

    affinePart[0][0] = warpingTransform[0][0];
    affinePart[0][1] = warpingTransform[0][1];
    affinePart[1][0] = warpingTransform[1][0];
    affinePart[1][1] = warpingTransform[1][1];

    eigen(affinePart, eigenValues);
    return eigenValues(1)/eigenValues(0);
}

float Measurement::measureStabilityScore(const std::vector<float>& measuredRotations,
    const std::vector<float>& measuredXTranslations, const std::vector<float>& measuredYTranslations)
{
    float stableRotationsEnergy = 0.0, stableXTranslationsEnergy = 0.0, stableYTranslationsEnergy = 0.0;
    float totalRotationsEnergy = 0.0, totalXTranslationsEnergy = 0.0, totalYTranslationsEnergy = 0.0;
    std::vector<float> rotationFrequencySpectrum, xTranslationsFrequencySpectrum, yTranslationsFrequencySpectrum;

    // checking whether number of measurements is bigger or equal to the required minimum of 10 measurements
    if(measuredRotations.size() < 10 || measuredXTranslations.size() < 10 || measuredYTranslations.size() < 10)
        return 1;

    // discrete Fourier transforms
    dft(measuredRotations, rotationFrequencySpectrum);
    dft(measuredXTranslations, xTranslationsFrequencySpectrum);
    dft(measuredYTranslations, yTranslationsFrequencySpectrum);

    for(unsigned int i = 1; (float)i < (float)rotationFrequencySpectrum.size()/2.0; i += 2)
        stableRotationsEnergy += pow(rotationFrequencySpectrum[i], 2) + pow(rotationFrequencySpectrum[i+1], 2);

    for(unsigned int i = 1; (float)i < (float)xTranslationsFrequencySpectrum.size()/2.0; i += 2)
        stableXTranslationsEnergy += pow(xTranslationsFrequencySpectrum[i], 2) + pow(xTranslationsFrequencySpectrum[i+1], 2);

    for(unsigned int i = 1; (float)i < (float)yTranslationsFrequencySpectrum.size()/2.0; i += 2)
        stableYTranslationsEnergy += pow(yTranslationsFrequencySpectrum[i], 2) + pow(yTranslationsFrequencySpectrum[i+1], 2);

    for(unsigned int i = 0; i < rotationFrequencySpectrum.size(); i++)
        totalRotationsEnergy += pow(rotationFrequencySpectrum[i], 2);

    for(unsigned int i = 0; i < xTranslationsFrequencySpectrum.size(); i++)
        totalXTranslationsEnergy += pow(xTranslationsFrequencySpectrum[i], 2);

    for(unsigned int i = 0; i < yTranslationsFrequencySpectrum.size(); i++)
        totalYTranslationsEnergy += pow(yTranslationsFrequencySpectrum[i], 2);

    rotationStabilityScore = stableRotationsEnergy / totalRotationsEnergy;
    xTranslationStabilityScore = stableXTranslationsEnergy / totalXTranslationsEnergy;
    yTranslationStabilityScore = stableYTranslationsEnergy / totalYTranslationsEnergy;

    return std::min(std::min(rotationStabilityScore, xTranslationStabilityScore), yTranslationStabilityScore);
}

void Measurement::extractAffineTransformParametersFromAffineMatrix(const Mat_<float>& affineTransformMatrix,
                                                      float& xTranslation, float& yTranslation,
                                                      float& scale, float& theta)
{
    xTranslation = affineTransformMatrix[0][2];
    yTranslation = affineTransformMatrix[1][2];
    float a = affineTransformMatrix[0][0], b = affineTransformMatrix[1][0];
    theta = atan2(b, a);
    scale = a/cos(theta);
}

// function handles only grayscale images, whose all pixels are stored as one byte
// there can be problem if image has full color rgb format
void Measurement::removeEntirelyBlackPixelsFromGrayScaleImage(Mat& grayScaleImage)
{
    unsigned int grayScaleImageWidth = grayScaleImage.cols;
    unsigned int grayScaleImageHeight = grayScaleImage.rows;

    for(unsigned int i = 0; i < grayScaleImageHeight; i++)
        for(unsigned int j = 0; j < grayScaleImageWidth; j++)
            if(grayScaleImage.at<char>(i, j) == 0)
                grayScaleImage.at<char>(i, j) = 1;
}
