#ifndef _STREAMRETRIEVE_H_
#define _STREAMRETRIEVE_H_

#include "Infra/Thread.h"
#include "GenICam/StreamSource.h"
#include "tool/Conf.h"

class StreamRetrieve : public Dahua::Infra::CThread {
public:
    StreamRetrieve(Dahua::GenICam::IStreamSourcePtr &streamSptr);

    bool start();

    void stop();

    cv::Mat getMatImage() const {
        return matImage_;
    }

    bool getCalReady() const {
        return calMatReady_;
    }

    double getFrameFps() const {
        return frameFps_;
    }

    void setCalReady(bool iReady) {
        calMatReady_ = iReady;
    }

    int getLostCameraCNT() const {
        return lostCameraCNT_;
    }

private:
    bool calMatReady_;
    bool showMatReady_;
    bool m_isLoop;
    int lostCameraCNT_ = 0;
    double frameFps_ = 0;
    cv::Mat originImage_;
    cv::Mat matImage_;
    Dahua::GenICam::IStreamSourcePtr m_streamSptr;

    void threadProc();

    void calFrameFps();

    static void imageConvert(Dahua::GenICam::CFrame &input, cv::Mat &originPut, cv::Mat &output);
};

typedef Dahua::Memory::TSharedPtr<StreamRetrieve> StreamRetrievePtr;


#endif
