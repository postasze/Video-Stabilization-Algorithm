TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

#OPENCV_SDK_DIR = C:\Users\pawel\Desktop\openCV_3_2_0_binaries_for_Qt\install
#OPENCV_INCLUDE_PATH = $$OPENCV_SDK_DIR\include
#OPENCV_LIB_PATH = $$OPENCV_SDK_DIR\x86\mingw\lib

#INCLUDEPATH += $$OPENCV_INCLUDE_PATH
#LIBS += $$join(OPENCV_LIB_PATH,,-L,)

#LIBS += -lopencv_core320
#LIBS += -lopencv_highgui320
#LIBS += -lopencv_features2d320
#LIBS += -lopencv_imgproc320
#LIBS += -lopencv_video320
#LIBS += -lopencv_videoio320

OPENCV_LIB_PATH=/home/pawel/opencv/build/lib
OPENCV_INCLUDE_PATH=/usr/local/include/opencv4

LIBS += $$join(OPENCV_LIB_PATH,,-L ,)
LIBS += -lopencv_video -lopencv_videoio -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_core -lopencv_features2d

INCLUDEPATH += $$OPENCV_INCLUDE_PATH

SOURCES += main.cpp
