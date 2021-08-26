#include "streamRetrieve.h"

StreamRetrieve::StreamRetrieve(Dahua::GenICam::IStreamSourcePtr &streamSptr) : CThread("streamRetrieve") {
    m_isLoop = false;
    calMatReady_ = false;
    showMatReady_ = false;
    m_streamSptr = streamSptr;
}

bool StreamRetrieve::start() {
    m_isLoop = true;
    return createThread();
}

void StreamRetrieve::stop() {
    m_isLoop = false;
    m_streamSptr.reset();
}

void StreamRetrieve::imageConvert(Dahua::GenICam::CFrame &input, cv::Mat &originPut, cv::Mat &output) {

    int imageFormat;

    //imageConvert函数测试区域边缘>>>>
    cv::Mat rawImageRGB(input.getImageHeight(), input.getImageWidth(), CV_8UC1, (void *) input.getImage());
    originPut = rawImageRGB;
    switch (input.getImagePixelFormat()) {
        case Dahua::GenICam::gvspPixelBayGR8:
            imageFormat = cv::COLOR_BayerGR2RGB;
            break;
        case Dahua::GenICam::gvspPixelBayRG8:
            imageFormat = cv::COLOR_BayerRG2RGB;
            break;
        case Dahua::GenICam::gvspPixelBayGB8:
            imageFormat = cv::COLOR_BayerGB2RGB;
            break;
        case Dahua::GenICam::gvspPixelBayBG8:
            imageFormat = cv::COLOR_BayerBG2RGB;
            break;
        default:
            imageFormat = cv::COLOR_BayerGR2RGB;
            break;
    }
    //<<<<imageConvert函数测试区域边缘1			2.3GHzCPU:1e-6s
    //imageConvert函数测试区域边缘>>>>
    cv::cvtColor(rawImageRGB, output, imageFormat);
    //<<<<imageConvert函数测试区域边缘2			2.3GHzCPU:0.0008-0.003s
}

void StreamRetrieve::calFrameFps() {
    //frameFps_ = 0;
    static int frameCount = 0;
    static double lastTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000; // ms

    ++frameCount;

    if (frameCount >= 100) // 取固定帧数为100帧
    {
        double curTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000;
        frameFps_ = frameCount / (curTime - lastTime) * 1000;
        lastTime = curTime;
        frameCount = 0;
    }
}

void StreamRetrieve::threadProc() {
#ifdef _LINUX
    int lostNum = 0;
#endif
    static bool imageInit = true;
    showMatReady_ = calMatReady_ = false;
    while (m_isLoop) {
        Dahua::GenICam::CFrame frame;
        //获取一帧
        //double t = (double)cv::getTickCount();
        bool isSuccess = m_streamSptr->getFrame(frame, 100);
        if (!isSuccess) {
            LOG::error("getFrame  fail.");
#ifdef _LINUX
            lostNum++;
            if(lostNum > 3){
                popen("reboot","r");
            }
#endif
            lostCameraCNT_++;
            continue;
        } else {
            lostCameraCNT_ = 0;
        }
        //判断帧的有效性
        bool isValid = frame.valid();
        if (!isValid) {
            LOG::error("frame is invalid!");
            continue;
        }
        //第一次初始化则对MatImage的参数进行修改
        if (imageInit) {
            cv::Mat cImageBGR(frame.getImageHeight(), frame.getImageWidth(), CV_8UC3);
            matImage_ = cImageBGR.clone();
            imageInit = false;
        }
        //转码
        imageConvert(frame, originImage_, matImage_);
        //TODO:根据相机类型判断是否应该修改分辨率
        //resize(matImage_, matImage_, cv ::Size(1280, 1024));//修改分辨率
        this->calFrameFps();
        showMatReady_ = calMatReady_ = true;
    }
}
