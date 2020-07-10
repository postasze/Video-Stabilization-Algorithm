#include "videostabilizationalgorithm.h"

VideoStabilizationAlgorithm::VideoStabilizationAlgorithm()
{

}

// implementation of outliers elimination algorithm
// double ended queue is used for optimization
std::vector<MotionVector> VideoStabilizationAlgorithm::OutliersElimination(
        const std::vector<MotionVector>& matchedKeypoints, unsigned int NumberOfOutliersToRemove)
{
    std::deque<MotionVector> matchedKeypointsDeque (matchedKeypoints.begin(), matchedKeypoints.end());
    std::vector<MotionVector> newMatchedKeypoints;
    MotionVector meanVector, smallestVector, biggestVector;
    double firstDifference, secondDifference;

    std::sort(matchedKeypointsDeque.begin(), matchedKeypointsDeque.end(), MotionVector::magnitudesComparer);

    for(unsigned int i = 0; i < NumberOfOutliersToRemove/2; i++)
    {
        if(matchedKeypointsDeque.empty())
            break;

        smallestVector = matchedKeypointsDeque.front();
        meanVector = matchedKeypointsDeque[matchedKeypointsDeque.size()/2];
        biggestVector = matchedKeypointsDeque.back();
        firstDifference = meanVector.magnitude - smallestVector.magnitude;
        secondDifference = biggestVector.magnitude - meanVector.magnitude;

        if(firstDifference < secondDifference)
            matchedKeypointsDeque.pop_back();
        else // firstDifference >= secondDifference
            matchedKeypointsDeque.pop_front();
    }

    std::sort(matchedKeypointsDeque.begin(), matchedKeypointsDeque.end(), MotionVector::anglesComparer);

    for(unsigned int i = NumberOfOutliersToRemove/2; i < NumberOfOutliersToRemove; i++)
    {
        if(matchedKeypointsDeque.empty())
            break;

        smallestVector = matchedKeypointsDeque.front();
        meanVector = matchedKeypointsDeque[matchedKeypointsDeque.size()/2];
        biggestVector = matchedKeypointsDeque.back();
        firstDifference = meanVector.angle - smallestVector.angle;
        secondDifference = biggestVector.angle - meanVector.angle;

        if(firstDifference < secondDifference)
            matchedKeypointsDeque.pop_back();
        else // firstDifference >= secondDifference
            matchedKeypointsDeque.pop_front();
    }

    newMatchedKeypoints.assign(matchedKeypointsDeque.begin(), matchedKeypointsDeque.end());
    return newMatchedKeypoints;
}

void VideoStabilizationAlgorithm::filterKeypoints(
        const vector<MotionVector>& matchedKeypoints, vector<Point2f>& correspondingKeyPointsOfFirstFrame,
        vector<Point2f>& correspondingKeyPointsOfSecondFrame)
{
    vector<MotionVector> filteredMatchedKeypoints; // motion vector shows the motion between two matched keypoints in two consecutive frames

    filteredMatchedKeypoints = OutliersElimination(matchedKeypoints, OUTLIERS_DISCARD_PERCENTAGE*matchedKeypoints.size());
    correspondingKeyPointsOfFirstFrame.clear();
    for(unsigned int i = 0; i < filteredMatchedKeypoints.size(); i++)
        correspondingKeyPointsOfFirstFrame.push_back(filteredMatchedKeypoints[i].startPoint);
    correspondingKeyPointsOfSecondFrame.clear();
    for(unsigned int i = 0; i < filteredMatchedKeypoints.size(); i++)
        correspondingKeyPointsOfSecondFrame.push_back(filteredMatchedKeypoints[i].endPoint);
}

// function which removes keypoints on edges of image,
// so that every keypoint contains pixels in every direction within window radius
void VideoStabilizationAlgorithm::removeKeyPointsOnBordersOfImage(
        std::vector<Point2f>& keyPointsOfFrame, const Mat& image)
{
    int imageWidth = image.cols;
    int imageHeight = image.rows;

    std::vector<Point2f> filteredKeyPointsOfFrame;
    for(unsigned int i = 0; i < keyPointsOfFrame.size(); i++)
    {
        if (keyPointsOfFrame[i].x >= WINDOW_RADIUS &&
                keyPointsOfFrame[i].y >= WINDOW_RADIUS &&
                keyPointsOfFrame[i].x < imageWidth - WINDOW_RADIUS &&
                keyPointsOfFrame[i].y < imageHeight - WINDOW_RADIUS)
            filteredKeyPointsOfFrame.push_back(keyPointsOfFrame[i]);
    }
    keyPointsOfFrame.assign(filteredKeyPointsOfFrame.begin(), filteredKeyPointsOfFrame.end());
}

// function assumes that every keypoint contains pixels in every direction within window radius
double VideoStabilizationAlgorithm::calculateDifferenceBetweenKeyPoints(
        const Mat& firstGrayFrame, const Mat& secondGrayFrame,
        const Point2f& firstKeyPoint, const Point2f& secondKeyPoint)
{
    double sumOfSquaredErrors = 0; // SSE or SSD (sum of squared differences)

    for(int i = -WINDOW_RADIUS; i <= WINDOW_RADIUS; i++)
        for(int j = -WINDOW_RADIUS; j <= WINDOW_RADIUS; j++)
            sumOfSquaredErrors += pow(firstGrayFrame.at<char>(firstKeyPoint.y+j, firstKeyPoint.x+i) - secondGrayFrame.at<char>(secondKeyPoint.y+j, secondKeyPoint.x+i), 2);

    sumOfSquaredErrors /= 2 * WINDOW_RADIUS + 1;

    return sumOfSquaredErrors;
}

void VideoStabilizationAlgorithm::findBestMatchesBetweenTwoFrames(
        const Mat& firstGrayFrame, const Mat& secondGrayFrame,
        const vector<Point2f>& keyPointsOfFirstFrame,
        const vector<Point2f>& keyPointsOfSecondFrame,
        vector<MotionVector>& matchedKeypoints)
{
    Point2f matchedKeyPoint;
    double minimalDifference, differenceBetweenKeyPoints;

    matchedKeypoints.clear();

    for(unsigned int i = 0; i < keyPointsOfFirstFrame.size(); i++)
    {
        minimalDifference = DBL_MAX;
        for(unsigned int j = 0; j < keyPointsOfSecondFrame.size(); j++)
        {
            differenceBetweenKeyPoints = calculateDifferenceBetweenKeyPoints(firstGrayFrame, secondGrayFrame, keyPointsOfFirstFrame[i], keyPointsOfSecondFrame[j]);
            if(minimalDifference > differenceBetweenKeyPoints)
            {
                matchedKeyPoint = keyPointsOfSecondFrame[j];
                minimalDifference = differenceBetweenKeyPoints;
            }
        }
        matchedKeypoints.push_back(*(new MotionVector(keyPointsOfFirstFrame[i], matchedKeyPoint)));
    }
}

void VideoStabilizationAlgorithm::resetTransformParameters(float transformParameters[4])
{
    for(unsigned int i = 0; i < 4; i++)
        if(i == 2) // Scale parameter
            transformParameters[i] = 1.0f;
        else // x, y translations, theta parameters
            transformParameters[i] = 0.0f;
}

void VideoStabilizationAlgorithm::smoothTransformParameters(float transformParameters[4])
{
    for(unsigned int i = 0; i < 4; i++)
        if(i == 2) // Scale parameter
            transformParameters[i] = ALPHA * transformParameters[i] + 1.0f - ALPHA;
        else // x, y translations, theta parameters
            transformParameters[i] = ALPHA * transformParameters[i];
}

void VideoStabilizationAlgorithm::extractAffineTransformParametersFromAffineMatrix(
        const Mat_<float>& affineTransformMatrix, float transformParameters[4])
{
    transformParameters[0] = affineTransformMatrix[0][2];
    transformParameters[1] = affineTransformMatrix[1][2];
    float a = affineTransformMatrix[0][0], b = affineTransformMatrix[1][0];
    transformParameters[3] = atan2(b, a);
    transformParameters[2] = a/cos(transformParameters[3]);
}

Mat_<float> VideoStabilizationAlgorithm::createAffineTransformFromTransformParameters(float transformParameters[4])
{
    Mat_<float> affineTransform = Mat::eye(3, 3, CV_32FC1); //affine 2x3 in a 3x3 matrix
    float xTranslation = transformParameters[0];
    float yTranslation = transformParameters[1];
    float scale = transformParameters[2];
    float theta = transformParameters[3];

    affineTransform[0][0] = scale * cos(theta);
    affineTransform[0][1] = -scale * sin(theta);
    affineTransform[0][2] = xTranslation;
    affineTransform[1][0] = scale * sin(theta);
    affineTransform[1][1] = scale * cos(theta);
    affineTransform[1][2] = yTranslation;
    affineTransform[2][0] = 0;
    affineTransform[2][1] = 0;
    affineTransform[2][2] = 1;

    return affineTransform;
}

void VideoStabilizationAlgorithm::paintKeyPointsOnFrame(const Mat& originalFrame,
                                                        const vector<Point2f>& correspondingKeyPointsOfFirstFrame,
                                                        const vector<Point2f>& correspondingKeyPointsOfSecondFrame)
{
    for (unsigned int i = 0; i < correspondingKeyPointsOfFirstFrame.size(); ++i)
        circle(originalFrame, correspondingKeyPointsOfFirstFrame[i], 3, Scalar(0,255,0), cv::FILLED);
    for (unsigned int i = 0; i < correspondingKeyPointsOfSecondFrame.size(); ++i)
        circle(originalFrame, correspondingKeyPointsOfSecondFrame[i], 3, Scalar(255,0,0), cv::FILLED);
}

bool VideoStabilizationAlgorithm::isTransformInvalid(const Mat_<float> &transformMatrix)
{
    if(transformMatrix.empty())
        return true;
    if(transformMatrix.cols < 2)
        return true;
    if(transformMatrix.rows < 2)
        return true;

    return false;
}

bool VideoStabilizationAlgorithm::isTransformErroneous(float transformParameters[4])
{
    if(!(-ERRONEOUS_X_TRANSLATION_THRESHOLD < transformParameters[0] &&
         transformParameters[0] < ERRONEOUS_X_TRANSLATION_THRESHOLD))
        return true;
    if(!(-ERRONEOUS_Y_TRANSLATION_THRESHOLD < transformParameters[1] &&
         transformParameters[1] < ERRONEOUS_Y_TRANSLATION_THRESHOLD))
        return true;
    if(!(1.0f - ERRONEOUS_SCALE_THRESHOLD < transformParameters[2] &&
         transformParameters[2] < 1.0f + ERRONEOUS_SCALE_THRESHOLD))
        return true;
    if(!(-ERRONEOUS_ROTATION_THRESHOLD < transformParameters[3] &&
         transformParameters[3] < ERRONEOUS_ROTATION_THRESHOLD))
        return true;

    return false;
}

bool VideoStabilizationAlgorithm::isTransformExaggerated(float transformParameters[])
{
    if(!(-ACCEPTABLE_X_TRANSLATION_THRESHOLD < transformParameters[0] &&
         transformParameters[0] < ACCEPTABLE_X_TRANSLATION_THRESHOLD))
        return true;
    if(!(-ACCEPTABLE_Y_TRANSLATION_THRESHOLD < transformParameters[1] &&
         transformParameters[1] < ACCEPTABLE_Y_TRANSLATION_THRESHOLD))
        return true;
    if(!(1.0f - ACCEPTABLE_SCALE_THRESHOLD < transformParameters[2] &&
         transformParameters[2] < 1.0f + ACCEPTABLE_SCALE_THRESHOLD))
        return true;
    if(!(-ACCEPTABLE_ROTATION_THRESHOLD < transformParameters[3] &&
         transformParameters[3] < ACCEPTABLE_ROTATION_THRESHOLD))
        return true;

    return false;
}

void VideoStabilizationAlgorithm::truncateTransformParameters(float transformParameters[])
{
    if(transformParameters[0] < -ACCEPTABLE_X_TRANSLATION_THRESHOLD)
        transformParameters[0] = -ACCEPTABLE_X_TRANSLATION_THRESHOLD;
    else if(transformParameters[0] > ACCEPTABLE_X_TRANSLATION_THRESHOLD)
        transformParameters[0] = ACCEPTABLE_X_TRANSLATION_THRESHOLD;

    if(transformParameters[1] < -ACCEPTABLE_Y_TRANSLATION_THRESHOLD)
        transformParameters[1] = -ACCEPTABLE_Y_TRANSLATION_THRESHOLD;
    else if(transformParameters[1] > ACCEPTABLE_Y_TRANSLATION_THRESHOLD)
        transformParameters[1] = ACCEPTABLE_Y_TRANSLATION_THRESHOLD;

    if(transformParameters[2] < 1.0f - ACCEPTABLE_SCALE_THRESHOLD)
        transformParameters[2] = 1.0f - ACCEPTABLE_SCALE_THRESHOLD;
    else if(transformParameters[2] > 1.0f + ACCEPTABLE_SCALE_THRESHOLD)
        transformParameters[2] = 1.0f + ACCEPTABLE_SCALE_THRESHOLD;

    if(transformParameters[3] < -ACCEPTABLE_ROTATION_THRESHOLD)
        transformParameters[3] = -ACCEPTABLE_ROTATION_THRESHOLD;
    else if(transformParameters[3] > ACCEPTABLE_ROTATION_THRESHOLD)
        transformParameters[3] = ACCEPTABLE_ROTATION_THRESHOLD;
}

void VideoStabilizationAlgorithm::removeEntirelyBlackPixelsFromImage(Mat &image)
{
    int imageWidth = image.cols;
    int imageHeight = image.rows;

    for(int i = 0; i < imageHeight; i++)
        for(int j = 0; j < imageWidth; j++)
            if(image.at<Vec3b>(i, j).val[0] == 0 && image.at<Vec3b>(i, j).val[1] == 0 && image.at<Vec3b>(i, j).val[2] == 0)
            {
                image.at<Vec3b>(i, j).val[0] = 5;
                image.at<Vec3b>(i, j).val[1] = 5;
                image.at<Vec3b>(i, j).val[2] = 5;
            }
}

bool VideoStabilizationAlgorithm::isPixelBlack(const Mat &frame, int x, int y)
{
    return frame.at<Vec3b>(y, x).val[0] == 0 && frame.at<Vec3b>(y, x).val[1] == 0 && frame.at<Vec3b>(y, x).val[2] == 0;
}

cv::Rect2i VideoStabilizationAlgorithm::findBiggestNonBlackRectangleWithinFrame(const Mat &frame)
{
    int frameWidth = frame.cols;
    int frameHeight = frame.rows;
    int leftBorder = 0, rightBorder = frameWidth, bottomBorder = frameHeight, topBorder = 0;
    bool leftBorderStop = false, rightBorderStop = false, topBorderStop = false, bottomBorderStop = false;
    bool containsBlackPixels = false;

    while(1)
    {
        if(leftBorder >= rightBorder)
            break;
        if(topBorder >= bottomBorder)
            break;
        if(leftBorderStop == true && rightBorderStop == true && topBorderStop == true && bottomBorderStop == true)
            break;

        containsBlackPixels = false;
        for(int i = topBorder; i < bottomBorder; i++)
            if(isPixelBlack(frame, leftBorder, i))
            {
                containsBlackPixels = true;
                break;
            }
        if(!containsBlackPixels)
            leftBorderStop = true;


        containsBlackPixels = false;
        for(int i = topBorder; i < bottomBorder; i++)
            if(isPixelBlack(frame, rightBorder, i))
            {
                containsBlackPixels = true;
                break;
            }
        if(!containsBlackPixels)
            rightBorderStop = true;

        containsBlackPixels = false;
        for(int j = leftBorder; j < rightBorder; j++)
            if(isPixelBlack(frame, j, topBorder))
            {
                containsBlackPixels = true;
                break;
            }
        if(!containsBlackPixels)
            topBorderStop = true;

        containsBlackPixels = false;
        for(int j = leftBorder; j < rightBorder; j++)
            if(isPixelBlack(frame, j, bottomBorder))
            {
                containsBlackPixels = true;
                break;
            }
        if(!containsBlackPixels)
            bottomBorderStop = true;

        if(!leftBorderStop)
            leftBorder++;
        if(!rightBorderStop)
            rightBorder--;
        if(!topBorderStop)
            topBorder++;
        if(!bottomBorderStop)
            bottomBorder--;
    }
    return cv::Rect2i(leftBorder, topBorder, rightBorder - leftBorder + 1, bottomBorder - topBorder + 1);
}

Rect2i VideoStabilizationAlgorithm::findIntersectingRectangle(Rect2i rectangleA, Rect2i rectangleB)
{
    int intersectingLeftX = std::max(rectangleA.x, rectangleB.x);
    int intersectingUpperY = std::max(rectangleA.y, rectangleB.y);
    int intersectingRightX = std::min(rectangleA.x + rectangleA.width, rectangleB.x + rectangleB.width);
    int intersectingLowerY = std::min(rectangleA.y + rectangleA.height, rectangleB.y + rectangleB.height);

    if(intersectingLeftX > intersectingRightX)
        return cv::Rect2i(); // empty rectangle
    if(intersectingUpperY > intersectingLowerY)
        return cv::Rect2i(); // empty rectangle

    return cv::Rect2i(intersectingLeftX, intersectingUpperY, intersectingRightX - intersectingLeftX,
                      intersectingLowerY - intersectingUpperY);
}

void VideoStabilizationAlgorithm::truncateFrameToSmallerRectangle(Mat &frame, cv::Rect2i newFrameRectangle, const Mat_<float> &transformMatrix)
{
    Mat newFrame(newFrameRectangle.height, newFrameRectangle.width, frame.type());
    Mat newFrameResized;
    for(int i = 0; i < newFrameRectangle.height; i++)
        for(int j = 0; j < newFrameRectangle.width; j++)
            newFrame.at<Vec3b>(i, j) = frame.at<Vec3b>(i + newFrameRectangle.y, j + newFrameRectangle.x);

    cv::resize(newFrame, newFrameResized, cv::Size(frame.cols, frame.rows));

    newFrameResized.copyTo(frame);
}

void VideoStabilizationAlgorithm::processAlgorithm()
{
    cv::VideoCapture videoCapture;
    cv::Mat frame, originalFrame, originalFrameWarped, firstGrayFrame, secondGrayFrame, outputFrame;
    std::vector<Point2f> keyPointsOfFirstFrame, keyPointsOfSecondFrame;
    std::vector<Point2f> correspondingKeyPointsOfFirstFrame, correspondingKeyPointsOfSecondFrame; // these structures are for correspondence between two sets of keypoints, 1st to 1st, 2nd to 2nd, 3rd to 3rd, etc
    std::vector<MotionVector> matchedKeypoints; // motion vector shows the motion between two matched keypoints in two consecutive frames
    cv::Mat_<float> affineTransform, inversedAffineTransform, cumulativeTransform = Mat::eye(3,3,CV_32FC1);
    cv::Mat_<float> newRigidTransform, previousAffineTransform, rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix;
    float transformParameters[4], extractedTransformParameters[4];
    int frameNumber = 0;
    bool findingKeyPointsError = false, estimatingTransformError = false;
    String sourceVideoFileName, destinationVideoFileName, currentFolder;
    Measurement measurement;
    std::ofstream ofstream ("output", std::ofstream::out);
    int videoNumber = 0, numberOfVideos = 0;
    std::deque<cv::Mat_<float>> frameCorrectingTransforms;
    cv::Rect2i biggestNonBlackRectangleWithinFrame, biggestNonBlackRectangleWithinVideo;

    for (int folderIndex = 1; folderIndex <= 6; folderIndex++) {
        switch(folderIndex)
        {
        case 1:
            currentFolder = "Crowd";
            numberOfVideos = 23;
            videoNumber = 23;
            break;
        case 2:
            currentFolder = "Parallax";
            numberOfVideos = 18;
            videoNumber = 18;
            break;
        case 3:
            currentFolder = "QuickRotation";
            numberOfVideos = 29;
            videoNumber = 29;
            break;
        case 4:
            currentFolder = "Regular";
            numberOfVideos = 23;
            videoNumber = 0;
            break;
        case 5:
            currentFolder = "Running";
            numberOfVideos = 22;
            videoNumber = 0;
            break;
        case 6:
            currentFolder = "Zooming";
            numberOfVideos = 29;
            videoNumber = 0;
            break;
        }

        for(; videoNumber < numberOfVideos; videoNumber++)
        {
            sourceVideoFileName = currentFolder + "/" + std::to_string(videoNumber) + ".avi";
            destinationVideoFileName = currentFolder + "/" + "stabilized" + std::to_string(videoNumber) + ".avi";

            ofstream << "Stabilizing video number: " << videoNumber << " with name: " << sourceVideoFileName << std::endl;

            videoCapture.open(sourceVideoFileName);

            // Default resolution of the frame is obtained.The default resolution is system dependent.
            int frame_width = videoCapture.get(cv::CAP_PROP_FRAME_WIDTH);
            int frame_height = videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT);

            //biggestNonBlackRectangleWithinVideo = cv::Rect2i(0, 0, frame_width, frame_height);
            frameCorrectingTransforms.clear();
            //double fourcc_codec_info = videoCapture.get(CV_CAP_PROP_FOURCC);

            // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
            // Define the fps to be equal to 10. Also frame size is passed.

            VideoWriter videoWriter(destinationVideoFileName, cv::VideoWriter::fourcc('M','J','P','G'), 29.97, Size(frame_width,frame_height));

            measurement.startMeasurement();
            resetTransformParameters(transformParameters);

            frameNumber = 0;
            videoCapture >> frame;
            if(frame.empty())
                return ;
            frame.copyTo(originalFrame);
            frameNumber++;

            frameCorrectingTransforms.push_back(cv::Mat_<float>::eye(3,3));

            cvtColor(originalFrame, firstGrayFrame, cv::COLOR_BGR2GRAY);

            goodFeaturesToTrack(firstGrayFrame, keyPointsOfFirstFrame, NUMBER_OF_KEYPOINTS, QUALITY_LEVEL, MINIMUM_DISTANCE);
            removeKeyPointsOnBordersOfImage(keyPointsOfFirstFrame, firstGrayFrame);

            putText(originalFrame, "Frame: " + std::to_string(frameNumber), Point2f(20, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 2, LINE_AA);

            //imshow("Video Stabilizer", originalFrame);

            //if(waitKey(0) == 27)
            //    return 0;

            //videoWriter.write(originalFrame);

            while(videoCapture.isOpened())
            {
                videoCapture >> frame;
                if(frame.empty())
                    break;
                frame.copyTo(originalFrame);
                frameNumber++;

                //if(frameNumber == 100)
                //    break;

                findingKeyPointsError = false;
                estimatingTransformError = false;

                cvtColor(originalFrame, secondGrayFrame, cv::COLOR_BGR2GRAY); // convert RGB image into gray image with openCV function

                keyPointsOfFirstFrame.clear();
                goodFeaturesToTrack(firstGrayFrame, keyPointsOfFirstFrame, NUMBER_OF_KEYPOINTS, QUALITY_LEVEL, MINIMUM_DISTANCE);
                removeKeyPointsOnBordersOfImage(keyPointsOfFirstFrame, firstGrayFrame);

                keyPointsOfSecondFrame.clear();
                goodFeaturesToTrack(secondGrayFrame, keyPointsOfSecondFrame, NUMBER_OF_KEYPOINTS, QUALITY_LEVEL, MINIMUM_DISTANCE);
                removeKeyPointsOnBordersOfImage(keyPointsOfSecondFrame, secondGrayFrame);

                if(keyPointsOfFirstFrame.size() < MINIMUM_NUMBER_OF_KEYPOINTS_TO_FIND || keyPointsOfSecondFrame.size() < MINIMUM_NUMBER_OF_KEYPOINTS_TO_FIND)
                {
                    ofstream << "problem with finding enough keypoint features in frame " + std::to_string(frameNumber) << std::endl;
                    findingKeyPointsError = true;
                }

                if(!findingKeyPointsError)
                {
                    findBestMatchesBetweenTwoFrames(firstGrayFrame, secondGrayFrame, keyPointsOfFirstFrame, keyPointsOfSecondFrame, matchedKeypoints);
                    filterKeypoints(matchedKeypoints, correspondingKeyPointsOfFirstFrame, correspondingKeyPointsOfSecondFrame);
                    //paintKeyPointsOnFrame(originalFrame, correspondingKeyPointsOfFirstFrame, correspondingKeyPointsOfSecondFrame);
                    newRigidTransform = estimateRigidTransform(correspondingKeyPointsOfFirstFrame, correspondingKeyPointsOfSecondFrame, false);

                    if(isTransformInvalid(newRigidTransform)) // error with estimating transform between these 2 frames
                        ofstream << "Invalid estimated transform error in frame " + std::to_string(frameNumber) << std::endl;
                    else
                    {
                        affineTransform = Mat_<float>::eye(3,3);
                        newRigidTransform.copyTo(affineTransform.rowRange(0,2));
                        inversedAffineTransform = affineTransform.inv(DECOMP_SVD);
                        extractAffineTransformParametersFromAffineMatrix(inversedAffineTransform, extractedTransformParameters);
                        if(isTransformErroneous(extractedTransformParameters))
                            ofstream << "Erroneous transform parameters in frame " + std::to_string(frameNumber) << std::setprecision(2)
                                     << ", X: " << extractedTransformParameters[0] << ", Y: " << extractedTransformParameters[1] <<
                                        ", S: " << extractedTransformParameters[2] << ", R: " << extractedTransformParameters[3] << std::endl;
                        else
                        {
                            if(isTransformExaggerated(extractedTransformParameters))
                            {
                                ofstream << "Exaggerated transform parameters in frame " + std::to_string(frameNumber) << std::setprecision(2)
                                         << ", X: " << extractedTransformParameters[0] << ", Y: " << extractedTransformParameters[1] <<
                                            ", S: " << extractedTransformParameters[2] << ", R: " << extractedTransformParameters[3] << std::endl;
                                truncateTransformParameters(extractedTransformParameters);
                            }
                            std::copy_n(extractedTransformParameters, 4, transformParameters);
                        }
                    }
                }

                smoothTransformParameters(transformParameters);
                inversedAffineTransform = createAffineTransformFromTransformParameters(transformParameters);
                /*inversedAffineTransform[0][0] = ALPHA * inversedAffineTransform[0][0] + 1.0 - ALPHA;
                inversedAffineTransform[0][1] = ALPHA * inversedAffineTransform[0][1];
                inversedAffineTransform[0][2] = ALPHA * inversedAffineTransform[0][2];
                inversedAffineTransform[1][0] = ALPHA * inversedAffineTransform[1][0];
                inversedAffineTransform[1][1] = ALPHA * inversedAffineTransform[1][1] + 1.0 - ALPHA;
                inversedAffineTransform[1][2] = ALPHA * inversedAffineTransform[1][2];
                */
                //cumulativeTransform *= inversedAffineTransform;

                //secondGrayFrame.copyTo(firstGrayFrame);
                warpAffine(secondGrayFrame, firstGrayFrame, inversedAffineTransform.rowRange(0,2), Size());
                //keyPointsOfFirstFrame.clear();
                //keyPointsOfFirstFrame = keyPointsOfSecondFrame;
                //transform(keyPointsOfSecondFrame, keyPointsOfFirstFrame, inversedAffineTransform.rowRange(0,2));

                if(measurement.isMeasurementOn)
                    measurement.measureStabilizationQuality(inversedAffineTransform, secondGrayFrame);

                removeEntirelyBlackPixelsFromImage(originalFrame);
                warpAffine(originalFrame, originalFrameWarped, inversedAffineTransform.rowRange(0,2), Size());
                frameCorrectingTransforms.push_back(inversedAffineTransform.clone());

                //biggestNonBlackRectangleWithinFrame = findBiggestNonBlackRectangleWithinFrame(originalFrameWarped);
                //biggestNonBlackRectangleWithinVideo = findIntersectingRectangle(biggestNonBlackRectangleWithinFrame, biggestNonBlackRectangleWithinVideo);
                //truncateFrameToSmallerRectangle(originalFrameWarped, biggestNonBlackRectangle, inversedAffineTransform);

                putText(originalFrameWarped, "Frame: " + std::to_string(frameNumber), Point2f(20, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 2, LINE_AA);
                //paintKeyPointsOnFrame(originalFrameWarped, keyPointsOfFirstFrame, keyPointsOfSecondFrame);
                //imshow("Video Stabilizer", originalFrameWarped);
                //if(waitKey(0) == 27)
                //    return;

                videoWriter.write(originalFrameWarped);
            }

            if(measurement.isMeasurementOn)
            {
                measurement.finishMeasurement();

                ofstream << "Measurement of stabilizing video number: " << videoNumber << " with name: " << sourceVideoFileName << std::endl;
                ofstream << "Cropping ratio: " << measurement.croppingRatio << std::endl;
                ofstream << "Distortion value: " << measurement.distortionValue << std::endl;
                ofstream << "Stability score: " << measurement.stabilityScore << std::endl;
                ofstream << "X translation stability score: " << measurement.xTranslationStabilityScore << std::endl;
                ofstream << "Y translation stability score: " << measurement.yTranslationStabilityScore << std::endl;
                ofstream << "Rotation stability score: " << measurement.rotationStabilityScore << std::endl;
            }

            /*
            videoCapture.release();
            videoCapture.open(sourceVideoFileName);
            VideoWriter videoWriter(destinationVideoFileName, cv::VideoWriter::fourcc('M','J','P','G'), 29.97, Size(frame_width,frame_height));
            frameNumber = 0;

            while(videoCapture.isOpened())
            {
                videoCapture >> frame;
                if(frame.empty())
                    break;
                frame.copyTo(originalFrame);
                frameNumber++;

                if(frameNumber == 99)
                    break;

                warpAffine(originalFrame, originalFrameWarped, frameCorrectingTransforms.front().rowRange(0,2), Size());
                truncateFrameToSmallerRectangle(originalFrameWarped, biggestNonBlackRectangleWithinVideo, frameCorrectingTransforms.front());
                frameCorrectingTransforms.pop_front();

                putText(originalFrameWarped, "Frame: " + std::to_string(frameNumber), Point2f(20, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 2, LINE_AA);
                videoWriter.write(originalFrameWarped);
                videoWriter.release();
            }*/
        }
    }

    ofstream.close();
    destroyAllWindows(); // close all the windows
}
