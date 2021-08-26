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
#define CODE_SET_NAME        "../visionData/codeSet.yml"                            //���������ļ���
#define SAVE_AVI_PATH       "../visionData/AVISave/"
#define SAVE_IMAGE_PATH     "../visionData/imgSave/"
#define SAVE_LOG_PATH       "../log/"
#define BUFF_PARA_NAME        "../visionData/buffPara.yml"                      //װ�װ����ݴ洢�ĵ�ַ
#define NEW_BUFF_PARA_NAME        "../visionData/newBuffPara.yml"                      //�°�������ݴ洢�ĵ�ַ
#define ARMOR_DATA_NAME        "../visionData/armorData.yml"                             //װ�װ������ļ���
#define INSTALL_DATA_NAME        "../visionData/installData.yml"                         //��װ��Ϣ�����ļ���
#define ARMOR_PARA_PATH        "../visionData/armorPara.yml"        //װ�װ����ݴ洢�ĵ�ַ

int main() {
#ifdef _LINUX
    int lostNum = 0;
#endif
    LOG::setLevel("debug");

    //�˳����봥����
    signal(SIGINT, Util::exitHandler);
    signal(SIGTERM, Util::exitHandler);

    //������־�洢λ��
    FileOperation::createDirectory(SAVE_LOG_PATH);
    string logPath = SAVE_LOG_PATH + to_string(FileOperation::getFileSizeInOrder(SAVE_LOG_PATH, ".log")) + ".log";
    LOG::setDestination(logPath);

    //��ȡ��������
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
        //�����쳣��ȫ��ʹ��Ĭ�ϲ���
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
    //�򿪹�ҵ���
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

    // ��ӡ���������Ϣ��key, ��������Ϣ, �ͺ�, ���кţ�
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
    //�����豸
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
    //����������
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
    //��������趨
    ModifyCamera::modifyCameraInit(cameraSptr);
    LOG::info("Modify camera success!");
    //��ʼ�������߳�
    Dahua::Memory::TSharedPtr<StreamRetrieve> streamThreadSptr(new StreamRetrieve(streamPtr));
    if (NULL == streamThreadSptr) {
        LOG::error("create stream thread obj failed.");
        return 0;
    }
    LOG::info("create stream thread obj success!");
    //��ʼ�����߳�
    streamThreadSptr->start();

#ifdef _LINUX
    //��ʼ�������߳�
    Dahua::Memory::TSharedPtr<Serial> serialSptr(new Serial());
    if (NULL == serialSptr) {
        LOG::info("create serial thread obj failed.");
        return 0;
    }
    //��ʼ�����߳�
    serialSptr->start();
    LOG::info("Strat getting image!");
    //��ʼ�������߳�
    Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel(streamThreadSptr, serialSptr, cameraSptr->getSerialNumber()));
#else
    //���������߳�
    Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel(streamThreadSptr, cameraSptr->getSerialNumber()));
#endif
    if (nullptr == decisionLevelSptr) {
        LOG::error("create stream thread obj failed.");
        return 0;
    }
    AutoSaveSample autoSaveSample(decisionLevelSptr, CodeSet::getUseAutoSaveSampleMode());
    cv::Mat saveImg;
    static DistinguishMode distinguishMode = TX2_STOP;
    //������Ƭ�����趨
    FileOperation::createDirectory(SAVE_IMAGE_PATH);
    uint32_t saveNumber = FileOperation::getFileSizeInOrder(SAVE_IMAGE_PATH, ".jpg");
    //������Ƶ�����趨
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
    //��ʼ�����߳�
    decisionLevelSptr->start();
    LOG::info("Start decision thread!");
    //��ʼ��ѭ��
    while (true) {
        //�Զ������ع�ʱ��
        //ModifyCamera::autoModifyExposureTime(streamThreadSptr->getMatImage().clone(), CodeSet::getGrayAverage(), decisionLevelSptr->getTargetCenter(), decisionLevelSptr->getTargetSize(), decisionLevelSptr->getDistinguishMode(), cameraSptr);

        //ͼƬ����׼����ϲ�����ʾ
        int keyASCII = 0;
        if (decisionLevelSptr->getCalReady()) {
            cv::Mat lin = streamThreadSptr->getMatImage();
            decisionLevelSptr->setOutputImage(&lin);
            for (int i = 0; i < 4; i++) {                                               //���Ʒ���������ͼƬ��
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
                /*���Դ���ʹ��:��Ҫ��ʾ��ͼ��clone��DEBGMAT CRETE BY BUFF*/
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
        if (keyASCII == 'a')                //a����С����
            ModifyCamera::modifyCameraGainRaw(cameraSptr, -0.1, ADD_VALUE);
        else if (keyASCII == 'd')           //d����������
            ModifyCamera::modifyCameraGainRaw(cameraSptr, 0.1, ADD_VALUE);
        else if (keyASCII == 'q')           //q����С٤��
            ModifyCamera::modifyCameraGamma(cameraSptr, -0.1, ADD_VALUE);
        else if (keyASCII == 'e')           //e������٤��
            ModifyCamera::modifyCameraGamma(cameraSptr, 0.1, ADD_VALUE);
        else if (keyASCII == 'w')           //w�������ع�ʱ��
            ModifyCamera::modifyCameraExposureTime(cameraSptr, 100, ADD_VALUE);
        else if (keyASCII == 's')           //s����С�ع�ʱ��
            ModifyCamera::modifyCameraExposureTime(cameraSptr, -100, ADD_VALUE);
        else if (keyASCII == 'c') {         //c����ͼ
            saveImg = streamThreadSptr->getMatImage().clone();
            cv::namedWindow("saveImg", 0);
            cv::imshow("saveImg", saveImg);
        } else if (keyASCII == 'v') {       //v�������ͼ
            string imgName = SAVE_IMAGE_PATH + to_string(saveNumber) + ".jpg";
            imwrite(imgName, saveImg);
            LOG::info("save image to " + imgName);
            ++saveNumber;
        } else if (keyASCII == 'z') {       //z���궨���
            CCalibration::cameraCali();
        } else if (keyASCII == 'x') {       //x�����ļ������������
            ModifyCamera::modifyCameraAutoSetFromFile(cameraSptr, decisionLevelSptr->getDistinguishMode());
        } else if (keyASCII == 27) break;   //ESC�˳�

        //�����ʧ�˳�
        if (streamThreadSptr->getLostCameraCNT() > 50) break;
        //������Ƶ
        if (CodeSet::getTakePictureFlag() && streamThreadSptr->getCalReady()) {
            videoWriter->write(streamThreadSptr->getMatImage());
        }
    }
    //ֹͣ�����߳�
    decisionLevelSptr->stop();
#ifdef  _LINUX
    //ֹͣ�����߳�
    serialSptr->stop();
#endif
    //ֹͣ�����߳�
    streamThreadSptr->stop();
    //ֹͣ�������
    streamPtr->stopGrabbing();
    //�Ͽ��豸
    if (!cameraSptr->disConnect()) {
        LOG::info("disConnect camera failed");
    }
    LOG::info("disConnect successfully thread ID : " + to_string(Dahua::Infra::CThread::getCurrentThreadID()));

    //ʹ����Ƶ
#else
    //���������߳�
    Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel("undefined"));
    //��ʼ�����߳�
    decisionLevelSptr->start();
    AutoSaveSample autoSaveSample(decisionLevelSptr, CodeSet::getUseAutoSaveSampleMode());
    while (true) {
        autoSaveSample.saveSample();
        if (cv::waitKey(1) == 27) break;
    }
#endif
    return 0;
}
