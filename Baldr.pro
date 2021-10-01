TARGET = Baldr
CONFIG += console
CONFIG += c++11

INCLUDEPATH += /usr/local/include/opencv4 \
               /opt/DahuaTech/MVviewer/include \
               ./include/eigen-3.3.7 \
               ./Src


LIBS += -L/usr/local/lib \
        /usr/local/lib/libopencv_* \
        /opt/DahuaTech/MVviewer/lib/libMVSDK.so \
        /opt/DahuaTech/MVviewer/module/USBDriver/libu3v-core.a

SOURCES += \
    Src/angle/angleFactory.cpp \
    Src/angle/CoordinateSolver/ArmorCoordinateSolver.cpp \
    Src/angle/CoordinateSolver/BuffCoordinateSolver.cpp \
    Src/angle/KalmaPredict/KalmaPredict.cpp \
    Src/angle/PNP/PNPSolver.cpp \
    Src/armor/armorDistinguish.cpp \
    Src/armor/bpPredict/net.cpp \
    Src/buff/buffDistinguish.cpp \
    Src/camera/modifyCamera.cpp \
    Src/camera/streamRetrieve.cpp \
    Src/camera/calibration/cameraCalibration.cpp \
    Src/decisionLevel/decisionLevel.cpp \
    Src/serial/serial.cpp \
    Src/tool/autoSaveSample.cpp \
    Src/tool/fileOperation.cpp \
    Src/main.cpp \
    Src/buff/buffTest.cpp \
    Src/angle/KalmanPredict/KalmanPredict.cpp

HEADERS  += \
    Src/angle/angleFactory.h \
    Src/angle/CoordinateSolver/ArmorCoordinateSolver.h \
    Src/angle/CoordinateSolver/BuffCoordinateSolver.h \
    Src/angle/KalmaPredict/KalmaPredict.h \
    Src/angle/PNP/PNPSolver.h \
    Src/armor/armorDistinguish.h \
    Src/armor/bpPredict/net.h \
    Src/buff/buffDistinguish.h \
    Src/camera/modifyCamera.h \
    Src/camera/streamRetrieve.h \
    Src/camera/calibration/cameraCalibration.h \
    Src/decisionLevel/decisionLevel.h \
    Src/serial/CRC.h \
    Src/serial/serial.h \
    Src/tool/autoSaveSample.h \
    Src/tool/Conf.h \
    Src/tool/fileOperation.h \
    Src/tool/PointUtil.h \
    Src/tool/RMDefine.h \
    Src/tool/RMLOG.h \
    Src/tool/systemChoose.h \
    Src/buff/buffTest.h \
    Src/angle/KalmanPredict/KalmanPredict.h
