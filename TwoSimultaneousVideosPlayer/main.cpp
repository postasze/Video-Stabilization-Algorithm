#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>

int main()
{
    cv::VideoCapture firstVideoCapture, secondVideoCapture;
    cv::Mat firstVideoFrame, secondVideoFrame, combinedFrame;
    int videoNumber = 0, numberOfVideos = 0, frameNumber = 0;
    cv::String firstVideoFileName, secondVideoFileName, destinationVideoFileName, currentFolder;

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
            firstVideoFileName = currentFolder + "/" + std::to_string(videoNumber) + ".avi";
            secondVideoFileName = currentFolder + "/stabilized" + std::to_string(videoNumber) + ".avi";
            destinationVideoFileName = currentFolder + "/compared" + std::to_string(videoNumber) + ".avi";

            firstVideoCapture.open(firstVideoFileName);
            secondVideoCapture.open(firstVideoFileName);
            secondVideoCapture.open(secondVideoFileName);

            int firstVideoWidth = firstVideoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
            int firstVideoHeight = firstVideoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
            int secondVideoWidth = secondVideoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
            int secondVideoHeight = secondVideoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);

            double first_video_fourcc_codec_info = firstVideoCapture.get(CV_CAP_PROP_FOURCC);
            double second_video_fourcc_codec_info = secondVideoCapture.get(CV_CAP_PROP_FOURCC);

            cv::VideoWriter videoWriter(destinationVideoFileName, CV_FOURCC('M','J','P','G'), 29.97, cv::Size(firstVideoWidth, firstVideoHeight + secondVideoHeight));
            //cv::VideoWriter videoWriter(destinationVideoFileName, CV_FOURCC('M','J','P','G'), 29.97, cv::Size(firstVideoWidth + secondVideoWidth, firstVideoHeight));

            while(firstVideoCapture.isOpened() && secondVideoCapture.isOpened())
            {
                firstVideoCapture >> firstVideoFrame;
                if(firstVideoFrame.empty())
                    break;
                secondVideoCapture >> secondVideoFrame;
                if(secondVideoFrame.empty())
                    break;
                frameNumber++;

                //cv::hconcat(firstVideoFrame, secondVideoFrame, combinedFrame);
                cv::vconcat(firstVideoFrame, secondVideoFrame, combinedFrame);

                videoWriter.write(combinedFrame);

                cv::imshow("Two simultaneous videos display", combinedFrame);

                if(cv::waitKey(0) == 27)
                    break;
            }
            videoWriter.release();
            firstVideoCapture.release();
            secondVideoCapture.release();
        }
    }
    return 0;
}
