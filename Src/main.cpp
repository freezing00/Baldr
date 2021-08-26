#include "GenICam/System.h"
#include "camera/streamRetrieve.h"
#include "camera/modifyCamera.h"
#include "decisionLevel/decisionLevel.h"
#include "tool/autoSaveSample.h"
#include <csignal>

#ifdef _LINUX

#include "GenICam/StreamSource.h"
#include "serial/serial.h"
#include <unistd.h>

#else

#include <windows.h>

#endif

#define CAMERA_CONFIGURATION_DATA_NAME  "../visionData/cameraConfigurationData.yml"
#define CODE_SET_NAME        "../visionData/codeSet.yml"                            //代码设置文件名
#define SAVE_AVI_PATH       "../visionData/AVISave/"
#define SAVE_IMAGE_PATH     "../visionData/imgSave/"
#define SAVE_LOG_PATH       "../log/"
#define BUFF_PARA_NAME        "../visionData/buffPara.yml"                      //装甲板数据存储的地址
#define NEW_BUFF_PARA_NAME        "../visionData/newBuffPara.yml"                      //新版神符数据存储的地址
#define ARMOR_DATA_NAME        "../visionData/armorData.yml"                             //装甲板数据文件名
#define INSTALL_DATA_NAME        "../visionData/installData.yml"                         //安装信息数据文件名
#define ARMOR_PARA_PATH        "../visionData/armorPara.yml"        //装甲板数据存储的地址

int main() {
#ifdef _LINUX
    int lostNum = 0;
#endif
    LOG::setLevel("debug");

    //退出代码触发器
    signal(SIGINT, Util::exitHandler);
    signal(SIGTERM, Util::exitHandler);

    //设置日志存储位置
    FileOperation::createDirectory(SAVE_LOG_PATH);
    string logPath = SAVE_LOG_PATH + to_string(FileOperation::getFileSizeInOrder(SAVE_LOG_PATH, ".log")) + ".log";
    LOG::setDestination(logPath);

    //读取代码设置
    try {
        CodeSet::readCodeSet(CODE_SET_NAME);
        ArmorDataFactory::readArmor(ARMOR_DATA_NAME);
        BuffParaFactory::readBuffPara(BUFF_PARA_NAME);
        NewBuffParaFactory::readNewBuffPara(NEW_BUFF_PARA_NAME);
        ArmorParaFactory::readArmorPara(ARMOR_PARA_PATH);
        CameraConfigurationFactory::readCameraConfiguration(CAMERA_CONFIGURATION_DATA_NAME);
        InstallDataFactory::readInstallData(INSTALL_DATA_NAME);
    } catch (cv::Exception &e) {
        LOG::error(e.msg.substr(0, e.msg.length() - 1));
        //遇到异常则全部使用默认参数
        CodeSet::resetAllConfig();
        ArmorDataFactory::resetAllConfig();
        BuffParaFactory::resetAllConfig();
        NewBuffParaFactory::resetAllConfig();
        ArmorParaFactory::resetAllConfig();
        CameraConfigurationFactory::resetAllConfig();
        InstallDataFactory::resetAllConfig();
        LOG::error("Catch exception while reading YML, reset all configuration!!");
    }

#if !TEST_VIDEO
    //打开工业相机
    Dahua::GenICam::ICameraPtr cameraSptr;
    ModifyCamera::CameraInitFlow cameraMode = ModifyCamera::INIT;
    Dahua::GenICam::CSystem &systemObj = Dahua::GenICam::CSystem::getInstance();;
    Dahua::Infra::TVector<Dahua::GenICam::ICameraPtr> vCameraPtrList;
    bool isDiscoverySuccess = false;
    while (cameraMode != ModifyCamera::START_SUCCESS) {
        switch (cameraMode) {
            case ModifyCamera::INIT:
                isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
                cameraMode = ModifyCamera::DISCOVERY_CAMERA;
                break;
            case ModifyCamera::DISCOVERY_CAMERA:
                if (!isDiscoverySuccess) {
                    LOG::error("discovery device fail.");
                    cameraMode = ModifyCamera::START_FAIL;
                } else if (vCameraPtrList.empty()) {
                    LOG::error("no devices.");
#ifdef _LINUX
                    lostNum++;
                    if (lostNum >= 3) {
                        popen("reboot","r");
                    }
#endif
                    cameraMode = ModifyCamera::START_FAIL;
                } else {
                    cameraMode = ModifyCamera::START_SUCCESS;
                }
                break;
            case ModifyCamera::START_FAIL:
                cameraMode = ModifyCamera::INIT;
                break;
        }
#ifdef _LINUX
        sleep(1);
#else
        Sleep(1000);
#endif
    }

    // 打印相机基本信息（key, 制造商信息, 型号, 序列号）
    for (std::size_t i = 0; i < vCameraPtrList.size(); i++) {
        cameraSptr = vCameraPtrList[i];
        LOG::info("Camera Info :");
        LOG::info("    key           = " + LOG::tostring(cameraSptr->getKey()));
        LOG::info("    vendor name   = " + LOG::tostring(cameraSptr->getVendorName()));
        LOG::info("    model         = " + LOG::tostring(cameraSptr->getModelName()));
        LOG::info("    serial number = " + LOG::tostring(cameraSptr->getSerialNumber()));
    }
    cameraSptr = vCameraPtrList[0];

    if (cameraSptr->isConnected()) {
        LOG::error("Camera is already connected!");
    }
    cameraSptr->disConnect();
    //连接设备
    while (!cameraSptr->connect()) {
        LOG::error("Connect camera failed.");
#ifdef _LINUX
        lostNum++;
        if (lostNum >= 10) {
            popen("reboot","r");
        }
#endif
    }
    LOG::info("Connected to camera!");
    //创建流对象
    Dahua::GenICam::IStreamSourcePtr streamPtr = systemObj.createStreamSource(cameraSptr);
    if (nullptr == streamPtr) {
        LOG::error("create stream obj  fail.");
        return 0;
    }
    LOG::info("create stream obj success!");
    streamPtr->stopGrabbing();
    bool isStartGrabbingSuccess = streamPtr->startGrabbing();
    if (!isStartGrabbingSuccess) {
        LOG::error("StartGrabbing  fail.");
    }
    LOG::info("StartGrabbing success!");
    //相机参数设定
    ModifyCamera::modifyCameraInit(cameraSptr);
    LOG::info("Modify camera success!");
    //初始化拉流线程
    Dahua::Memory::TSharedPtr<StreamRetrieve> streamThreadSptr(new StreamRetrieve(streamPtr));
    if (NULL == streamThreadSptr) {
        LOG::error("create stream thread obj failed.");
        return 0;
    }
    LOG::info("create stream thread obj success!");
    //开始拉流线程
    streamThreadSptr->start();

#ifdef _LINUX
    //初始化串口线程
    Dahua::Memory::TSharedPtr<Serial> serialSptr(new Serial());
    if (NULL == serialSptr) {
        LOG::info("create serial thread obj failed.");
        return 0;
    }
    //开始串口线程
    serialSptr->start();
    LOG::info("Strat getting image!");
    //初始化计算线程
    Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel(streamThreadSptr, serialSptr, cameraSptr->getSerialNumber()));
#else
    //创建计算线程
    Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel(streamThreadSptr, cameraSptr->getSerialNumber()));
#endif
    if (nullptr == decisionLevelSptr) {
        LOG::error("create stream thread obj failed.");
        return 0;
    }
    AutoSaveSample autoSaveSample(decisionLevelSptr, CodeSet::getUseAutoSaveSampleMode());
    cv::Mat saveImg;
    static DistinguishMode distinguishMode = TX2_STOP;
    //保存照片参数设定
    FileOperation::createDirectory(SAVE_IMAGE_PATH);
    uint32_t saveNumber = FileOperation::getFileSizeInOrder(SAVE_IMAGE_PATH, ".jpg");
    //保存视频参数设定
    while (!streamThreadSptr->getCalReady());
    cv::VideoWriter *videoWriter = nullptr;
    if (CodeSet::getTakePictureFlag()) {
        cout << CodeSet::getTakePictureFlag() << endl;
        FileOperation::createDirectory(SAVE_AVI_PATH);
        std::string saveName = SAVE_AVI_PATH + to_string(FileOperation::getFileSizeInOrder(SAVE_AVI_PATH, ".avi")) + ".avi";
        cv::Size saveSize = cv::Size(streamThreadSptr->getMatImage().cols, streamThreadSptr->getMatImage().rows);
        videoWriter = new cv::VideoWriter(saveName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 42, saveSize);
        LOG::info("Save avi to " + saveName);
    }
    //开始计算线程
    decisionLevelSptr->start();
    LOG::info("Start decision thread!");
    //开始主循环
    while (true) {
        //自动调整曝光时间
        //ModifyCamera::autoModifyExposureTime(streamThreadSptr->getMatImage().clone(), CodeSet::getGrayAverage(), decisionLevelSptr->getTargetCenter(), decisionLevelSptr->getTargetSize(), decisionLevelSptr->getDistinguishMode(), cameraSptr);

        //图片必须准备完毕才能显示
        int keyASCII = 0;
        if (decisionLevelSptr->getCalReady()) {
            cv::Mat lin = streamThreadSptr->getMatImage();
            decisionLevelSptr->setOutputImage(&lin);
            for (int i = 0; i < 4; i++) {                                               //绘制方框线条到图片上
                cv::line(decisionLevelSptr->getOutputImage(), decisionLevelSptr->getVertices()[i], decisionLevelSptr->getVertices()[(i + 1) % 4], CV_RGB(0, 255, 0), 2);
            }
            if (distinguishMode == TX2_DISTINGUISH_ARMOR) {
                cv::circle(decisionLevelSptr->getOutputImage(), cv::Point2f(960, 600), 5, cv::Scalar(0, 0, 255), -1, -1, 0);
                cv::circle(decisionLevelSptr->getOutputImage(), (decisionLevelSptr->getVertices()[0] + decisionLevelSptr->getVertices()[2]) / 2, 2, cv::Scalar(0, 255, 0), -1, -1, 0);
                cv::circle(decisionLevelSptr->getOutputImage(), decisionLevelSptr->getPredictPoint(), 2, cv::Scalar(255, 0, 255), -1, -1, 0);
            }
            if (distinguishMode == TX2_DISTINGUISH_BUFF || distinguishMode == TX2_DISTINGUISH_BIG_BUFF) {
               cv::circle(decisionLevelSptr->getOutputImage(), decisionLevelSptr->getBuffPoint(), 2, cv::Scalar(0, 0, 255), 2, 8, 0);
               cv::circle(decisionLevelSptr->getOutputImage(), decisionLevelSptr->getRPoint(), 8, cv::Scalar(0, 255, 0), 6, 8, 0);

            }
#ifdef  _LINUX
            if (CodeSet::getUseDebugGuiFlag()) {
                cv::namedWindow("outputImage", 0);
                cv::imshow("outputImage", decisionLevelSptr->getOutputImage());
                /*调试窗口使用:将要显示的图像clone给DEBGMAT CRETE BY BUFF*/
//                cv::namedWindow("DEBUGWINDOW", 0);
//                cv::namedWindow("DEBUGWINDOW1", 0);
//                if(!decisionLevelSptr->GETDEBUGMAT().empty())
//                    cv::imshow("DEBUGWINDOW", decisionLevelSptr->GETDEBUGMAT());
//                if(!decisionLevelSptr->GETDEBUGMAT1().empty())
//                    cv::imshow("DEBUGWINDOW1", decisionLevelSptr->GETDEBUGMAT1());
             // LOG::info("frame FPS is " + to_string(streamThreadSptr->getFrameFps()) + ", calculation FPS is " + to_string(decisionLevelSptr->getCalculationFps()));
            }

#else

            cv::namedWindow("outputImage", 0);
            cv::imshow("outputImage", decisionLevelSptr->getOutputImage());
            //LOG::info("frame FPS is " + to_string(streamThreadSptr->getFrameFps()) + ", calculation FPS is " + to_string(decisionLevelSptr->getCalculationFps()));
#endif
            autoSaveSample.saveSample();
            decisionLevelSptr->setCalReady(false);
        }
#ifdef  _LINUX
        if (serialSptr->getCalReady()) {
            distinguishMode = decisionLevelSptr->getDistinguishMode();
        }
#else
        distinguishMode = decisionLevelSptr->getDistinguishMode();
#endif
        keyASCII = cv::waitKey(1);
#ifdef  _LINUX
        if (serialSptr->isLost()) {
            keyASCII = 27;
        }
#endif
        if (keyASCII == 'a')                //a键减小增益
            ModifyCamera::modifyCameraGainRaw(cameraSptr, -0.1, ADD_VALUE);
        else if (keyASCII == 'd')           //d键增大增益
            ModifyCamera::modifyCameraGainRaw(cameraSptr, 0.1, ADD_VALUE);
        else if (keyASCII == 'q')           //q键减小伽马
            ModifyCamera::modifyCameraGamma(cameraSptr, -0.1, ADD_VALUE);
        else if (keyASCII == 'e')           //e键增大伽马
            ModifyCamera::modifyCameraGamma(cameraSptr, 0.1, ADD_VALUE);
        else if (keyASCII == 'w')           //w键增大曝光时间
            ModifyCamera::modifyCameraExposureTime(cameraSptr, 100, ADD_VALUE);
        else if (keyASCII == 's')           //s键减小曝光时间
            ModifyCamera::modifyCameraExposureTime(cameraSptr, -100, ADD_VALUE);
        else if (keyASCII == 'c') {         //c键截图
            saveImg = streamThreadSptr->getMatImage().clone();
            cv::namedWindow("saveImg", 0);
            cv::imshow("saveImg", saveImg);
        } else if (keyASCII == 'v') {       //v键保存截图
            string imgName = SAVE_IMAGE_PATH + to_string(saveNumber) + ".jpg";
            imwrite(imgName, saveImg);
            LOG::info("save image to " + imgName);
            ++saveNumber;
        } else if (keyASCII == 'z') {       //z键标定相机
            CCalibration::cameraCali();
        } else if (keyASCII == 'x') {       //x键从文件读入相机参数
            ModifyCamera::modifyCameraAutoSetFromFile(cameraSptr, decisionLevelSptr->getDistinguishMode());
        } else if (keyASCII == 27) break;   //ESC退出

        //相机丢失退出
        if (streamThreadSptr->getLostCameraCNT() > 50) break;
        //保存视频
        if (CodeSet::getTakePictureFlag() && streamThreadSptr->getCalReady()) {
            videoWriter->write(streamThreadSptr->getMatImage());
        }
    }
    //停止计算线程
    decisionLevelSptr->stop();
#ifdef  _LINUX
    //停止串口线程
    serialSptr->stop();
#endif
    //停止拉流线程
    streamThreadSptr->stop();
    //停止相机拉流
    streamPtr->stopGrabbing();
    //断开设备
    if (!cameraSptr->disConnect()) {
        LOG::info("disConnect camera failed");
    }
    LOG::info("disConnect successfully thread ID : " + to_string(Dahua::Infra::CThread::getCurrentThreadID()));

    //使用视频
#else
    //创建计算线程
    Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel("undefined"));
    //开始计算线程
    decisionLevelSptr->start();
    AutoSaveSample autoSaveSample(decisionLevelSptr, CodeSet::getUseAutoSaveSampleMode());
    while (true) {
        autoSaveSample.saveSample();
        if (cv::waitKey(1) == 27) break;
    }
#endif
    return 0;
}
