TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += QT_DEPRECATED_WARNINGS

OPENCV_LIB_PATH=/home/pawel/opencv/build/lib
OPENCV_INCLUDE_PATH=/usr/local/include/opencv4

LIBS += $$join(OPENCV_LIB_PATH,,-L ,)
LIBS += -lopencv_video -lopencv_videoio -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_core -lopencv_features2d

INCLUDEPATH += $$OPENCV_INCLUDE_PATH

SOURCES += main.cpp \
    motionvector.cpp \
    linearalgebrafunctions.cpp \
    measurement.cpp \
    videostabilizationalgorithm.cpp

HEADERS += \
    constants.h \
    motionvector.h \
    linearalgebrafunctions.h \
    measurement.h \
    videostabilizationalgorithm.h
