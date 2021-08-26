#include "camera/modifyCamera.h"

void ModifyCamera::setCameraResolution(const Dahua::GenICam::ICameraPtr &CameraSptr, int width, int height) {

    bool bRet;
    Dahua::GenICam::IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(CameraSptr);
    Dahua::GenICam::CIntNode intNode = sptrImageFormatControl->width();

    bRet = intNode.setValue(width);
    if (!bRet) {
        LOG::error("set width fail.");
    }

    intNode = sptrImageFormatControl->height();
    bRet = intNode.setValue(height);
    if (!bRet) {
        LOG::error("set height fail.");
    }
}

void ModifyCamera::modifyCameraExposureMode(Dahua::GenICam::ICameraPtr &cameraSptr, CameraModeSet exposureTimeMode) {
    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (nullptr == sptrAcquisitionControl) {
        return;
    }
    Dahua::Infra::CString exposureAutoValue;
    Dahua::GenICam::CEnumNode exposureAuto = sptrAcquisitionControl->exposureAuto();
    switch (exposureTimeMode) {
        case OFF:
            exposureAutoValue.operator=("Off");
            break;
        case ONCE:
            exposureAutoValue.operator=("Once");
            break;
        case CONTINUOUS:
            exposureAutoValue.operator=("Continuous");
            break;
    }
    if (exposureAuto.setValueBySymbol(exposureAutoValue)) {
        LOG::info("set exposure mode " + LOG::tostring(exposureAutoValue.data()) + " success!");
    } else {
        LOG::error("set exposure mode " + LOG::tostring(exposureAutoValue.data()) + " fail!");
    }
}

void ModifyCamera::modifyCameraExposureTime(Dahua::GenICam::ICameraPtr &cameraSptr, double exposureTimeSet, bool addFlag) {
    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (nullptr == sptrAcquisitionControl) {
        return;
    }
    double exposureTimeValue = 0.0;
    Dahua::GenICam::CDoubleNode exposureTime = sptrAcquisitionControl->exposureTime();
    exposureTime.getValue(exposureTimeValue);
    if (addFlag) {
        exposureTimeValue += exposureTimeSet;
        if (exposureTimeValue < 1.0)
            exposureTimeValue = 1.0;
        else if (exposureTimeValue > 10000.0)
            exposureTimeValue = 10000.0;
        exposureTime.setValue(exposureTimeValue);
    } else {
        if (exposureTimeSet < 1.0)
            exposureTimeSet = 1.0;
        else if (exposureTimeSet > 10000.0)
            exposureTimeSet = 10000.0;
        exposureTime.setValue(exposureTimeSet);
    }
    exposureTime.getValue(exposureTimeValue);
    //LOG::debug("after change ,exposureTime is " + std::to_string(exposureTimeValue));

}

void ModifyCamera::modifyCameraGainRaw(Dahua::GenICam::ICameraPtr &cameraSptr, double gainRawSet, bool addFlag) {
    Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(cameraSptr);
    if (nullptr == sptrAnalogControl) {
        return;
    }
    double gainRawValue = 0.0;
    Dahua::GenICam::CDoubleNode gainRaw = sptrAnalogControl->gainRaw();
    gainRaw.getValue(gainRawValue);
    if (addFlag) {
        gainRawValue += gainRawSet;
        if (gainRawValue < 1.0)
            gainRawValue = 1.0;
        else if (gainRawValue > 32.0)
            gainRawValue = 32.0;
        gainRaw.setValue(gainRawValue);
    } else {
        if (gainRawSet < 1.0)
            gainRawSet = 1.0;
        else if (gainRawSet > 32.0)
            gainRawSet = 32.0;
        gainRaw.setValue(gainRawSet);
    }
    gainRaw.getValue(gainRawValue);
    LOG::info("after change ,gainRaw is " + std::to_string(gainRawValue));
}

void ModifyCamera::modifyCameraGamma(Dahua::GenICam::ICameraPtr &cameraSptr, double gammaSet, bool addFlag) {
    Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(cameraSptr);
    if (nullptr == sptrAnalogControl) {
        return;
    }
    double gammaValue = 0.0;
    Dahua::GenICam::CDoubleNode gamma = sptrAnalogControl->gamma();
    gamma.getValue(gammaValue);
    if (addFlag) {
        gammaValue += gammaSet;
        if (gammaValue < 0.1)
            gammaValue = 0.1;
        else if (gammaValue > 4.0)
            gammaValue = 4.0;
        gamma.setValue(gammaValue);
    } else {
        if (gammaSet < 0.1)
            gammaSet = 0.1;
        else if (gammaSet > 4.0)
            gammaSet = 4.0;
        gamma.setValue(gammaSet);
    }
    gamma.getValue(gammaValue);
    LOG::info("after change ,gamma is " + std::to_string(gammaValue));
}

void ModifyCamera::modifyCameraAutoSetFromFile(Dahua::GenICam::ICameraPtr &cameraSptr, DistinguishMode distinguishMode) {
    //��������趨
    switch (distinguishMode) {
        case TX2_STOP:
        case TX2_DISTINGUISH_ARMOR:
        case TX2_DISTINGUISH_BUFF:
        case TX2_DISTINGUISH_BIG_BUFF:
            modifyCameraExposureTime(cameraSptr, CameraConfigurationFactory::getBuffCameraConfiguration().exposureValue, SET_VALUE);
            modifyCameraGainRaw(cameraSptr, CameraConfigurationFactory::getBuffCameraConfiguration().gainValue, SET_VALUE);
            modifyCameraGamma(cameraSptr, CameraConfigurationFactory::getBuffCameraConfiguration().gammaValue, SET_VALUE);
            break;
    }
}

void ModifyCamera::modifyCameraBalanceWhiteMode(Dahua::GenICam::ICameraPtr &cameraSptr, CameraModeSet balanceWhiteModeSet) {
    Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(cameraSptr);
    if (nullptr == sptrAnalogControl) {
        return;
    }
    //���Զ���ƽ��
    Dahua::GenICam::CEnumNode enumNode = sptrAnalogControl->balanceWhiteAuto();
    Dahua::Infra::CString value;
    switch (balanceWhiteModeSet) {
        case OFF:
            value.operator=("Off");
            break;
        case ONCE:
            value.operator=("Once");
            break;
        case CONTINUOUS:
            value.operator=("Continuous");
            break;
    }
    if (enumNode.setValueBySymbol(value)) {
        LOG::info("set BalanceWhite mode " + LOG::tostring(value.data()) + " success!");
    } else {
        LOG::error("set BalanceWhite mode " + LOG::tostring(value.data()) + " fail!");
    }
}

void ModifyCamera::modifyCameraBrightness(Dahua::GenICam::ICameraPtr &cameraSptr, int value) {
    Dahua::GenICam::IISPControlPtr ISPControl = Dahua::GenICam::CSystem::getInstance().createISPControl(cameraSptr);
    if (nullptr == ISPControl) {
        return;
    }
    Dahua::GenICam::CIntNode brightness = ISPControl->brightness();
    brightness.setValue(value);
    LOG::info("after change ,brightness is " + std::to_string(value));
}

void ModifyCamera::modifyCameraSharpnessAuto(Dahua::GenICam::ICameraPtr &cameraSptr) {
    Dahua::GenICam::IISPControlPtr ISPControl = Dahua::GenICam::CSystem::getInstance().createISPControl(cameraSptr);
    if (nullptr == ISPControl) {
        return;
    }
    Dahua::GenICam::CEnumNode enable = ISPControl->sharpnessEnable();
    enable.setValueBySymbol(Dahua::Infra::CString("On"));
    Dahua::GenICam::CBoolNode node = ISPControl->sharpnessAuto();
    node.setValue(true);
}


void ModifyCamera::autoModifyExposureTime(cv::Mat src, int grayAverage, const cv::Point2f &targetCenter, const cv::Size2f &targetSize, DistinguishMode distinguishMode, Dahua::GenICam::ICameraPtr &cameraSptr, int offset) {
    float scale = 3.0f;
    if (!targetSize.empty() && targetCenter.x > 0) {
        if (distinguishMode == TX2_DISTINGUISH_BUFF || distinguishMode == TX2_DISTINGUISH_BIG_BUFF) {
            //���ʶ��ģʽ�������������������Сƽ���Ҷ�
            grayAverage -= 5;
            scale = 10.0f;
        }
        int x1 = MAX(int(targetCenter.x - (targetSize.width * scale)), 0);
        int y1 = MAX(int(targetCenter.y - (targetSize.height * scale)), 0);
        cv::Point lu = cv::Point(x1, y1);                                    //�õ���lu��ROI���Ͻǵ����
        int x2 = MIN(int(targetCenter.x + (targetSize.width * scale)), src.cols);                            //������������ͼƬ����
        int y2 = MIN(int(targetCenter.y + (targetSize.height * scale)), src.rows);
        cv::Point rd = cv::Point(x2, y2);                                    //�õ���rd��ROI���½ǵ��յ�
        src = src(cv::Rect(lu, rd));
    }

    //�����׹�
    const int totalPixel = src.cols * src.rows * 3;
    const uchar *ptr_begin = src.data;                         //ͼƬ�����׵�ַ
    const uchar *ptr_end = src.data + totalPixel;     //ͼƬ����β��ַ
    float graySum = 0, grayAver = 0;

    const uchar maxLight = 250;
    static uchar B, G, R;
    //ѭ������ͼƬ
    while (ptr_begin != ptr_end) {
        //BGR��ͨ����ֵ��С���趨ֵ������ƽ���Ҷȣ����˰׹�
        B = *ptr_begin++;
        G = *ptr_begin++;
        R = *ptr_begin++;
        if (B < maxLight && G < maxLight && R < maxLight) {
            graySum = graySum + B + G + R;
        }
    }
    grayAver = graySum / totalPixel;

//    std::cout << "grayAver:" << grayAver << std::endl << std::endl;

    //����������ٵ���
    if (grayAver > grayAverage * 2.5) {
        modifyCameraExposureTime(cameraSptr, -200, ADD_VALUE);
        return;
    }
    if ((grayAver > (grayAverage + offset))) {
        modifyCameraExposureTime(cameraSptr, -50, ADD_VALUE);
    } else if ((grayAver < (grayAverage - offset))) {
        modifyCameraExposureTime(cameraSptr, 50, ADD_VALUE);
    }
}

void ModifyCamera::modifyCameraInit(Dahua::GenICam::ICameraPtr &cameraSptr) {
    //�ر�����ڲ����Զ��ع�
    modifyCameraExposureMode(cameraSptr, OFF);
    //���Զ���ƽ��
    modifyCameraBalanceWhiteMode(cameraSptr, CONTINUOUS);
    //�����������
    modifyCameraBrightness(cameraSptr, 50);
    //�����Զ����
    modifyCameraSharpnessAuto(cameraSptr);
    //���ļ�����gamma��gain
    modifyCameraAutoSetFromFile(cameraSptr);

    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(cameraSptr);
    double exposureTimeValue = 0.0;
    Dahua::GenICam::CDoubleNode exposureTime = sptrAcquisitionControl->exposureTime();
    exposureTime.getValue(exposureTimeValue);
    exposureTime.setValue(1000.0f);
    exposureTime.getValue(exposureTimeValue);
    //std::cout << "exposureTimeValue:" << exposureTimeValue << std::endl;
}

