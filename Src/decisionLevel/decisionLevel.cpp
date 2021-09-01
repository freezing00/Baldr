#include <serial/serial.h>
#include "decisionLevel.h"
#include <angle/KalmanPredict/KalmanPredict.h>

#define BUFF_MODE 
#define DISTINGUISH_MODE TX2_DISTINGUISH_ARMOR
#define DISTINGUISH_PART 1 //0Ϊ���ۣ�1ΪԲ��
#define ENEMY_COLOR_MODE ENEMY_RED
#define SHOOT_SPEED 26
#define CAR_TYPE INFANTRY

DecisionLevel::DecisionLevel(const string &serialNumber) : CThread("decisionLevel") {
    this->m_isLoop = false;
    this->calReady_ = false;
    this->stm32CmdData.pitchData.float_temp = 0.0f;
    this->stm32CmdData.yawData.float_temp = 0.0f;
    this->angleFactory_ = new AngleFactory(serialNumber);
}

DecisionLevel::DecisionLevel(StreamRetrievePtr &StreamData, const string &serialNumber) : DecisionLevel(serialNumber) {
    this->streamData_ = StreamData;
}

DecisionLevel::DecisionLevel(StreamRetrievePtr &StreamData, SerialPtr &SerialData, const string &serialNumber) : DecisionLevel(StreamData, serialNumber) {
    this->serialData_ = SerialData;
}

void DecisionLevel::sendDecisionCmd() {
#ifdef _LINUX
    serialData_->sendCmd(&_tx2Command);
#endif
}

void DecisionLevel::calFps() {
    static int count = 0;
    static double lastTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000; // ms
    ++count;
    // ȡ�̶�֡��Ϊ100֡����һ��
    if (count >= 100) {
        double curTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000;
        calculationFps_ = count / (curTime - lastTime) * 1000;
        lastTime = curTime;
        count = 0;
    }
}

//װ�װ���������
void DecisionLevel::saveSampleImage(ArmorDistinguish &armorDistinguish) {
    if (armorDistinguish.getSampleCaptureState()) {
        sampleData_ = armorDistinguish.getSampleData();
        captureState_ = true;
    } else {
        captureState_ = false;
    }
    armorDistinguish.setSampleCaptureState(false);
}

//�����������
void DecisionLevel::buffSaveSampleImage(BuffDistinguish &buffDistinguish) {
    if (buffDistinguish.getSampleCaptureState()) {
        if (!distinguishPart_) {
            sampleData_ = buffDistinguish.getSampleData();
        } else {
            sampleData_ = buffDistinguish.getCircleSampleData();
        }
        captureState_ = true;
    } else {
        captureState_ = false;
    }
    buffDistinguish.setSampleCaptureState(false);
}
cv::Mat DecisionLevel::GETDEBUGMAT(){
    return DEBUGMAT.clone();
}
//���ʶ�����
void DecisionLevel::buffDistinguishProcess(const cv::Mat &src) {
#ifdef BUFF_MODE
    cv::Mat inputImg = src;
    _buff.setMat(inputImg, (OwnColor)enemyColor_);
    rect_ = _buff.getTargetRect();
    //cout<<"rect"<<rect_.center<<endl;
    sameTargetFlag_ = _buff.getSameTargetFlag();
    R_2D_Center_ = _buff.getRCenter();
    _findR_Flag = _buff.getRFlag();
    //DEBUGMAT=_buff.getDEBUGMAT();
    DEBUGMAT1=_buff.getDEBUGMAT();
    DEBUGMAT=_buff.getBinaryMat();
#else
    rect_ = buffDistinguish_.process(src, (OwnColor) !enemyColor_);
    sameTargetFlag_ = buffDistinguish_.getSameTargetFlag();
    R_2D_Center_ = buffDistinguish_.getR_2D_Center();
    captureR_Flag_ = buffDistinguish_.getCaptureR_Flag();
    _findRectFlag = buffDistinguish_.getFindRectFlag();
#endif // !BUFF_MODE

    //����Ŀ�����ĵ㹩�Զ��ع�ʹ��
    targetCenter = R_2D_Center_;

    //�����������
    buffSaveSampleImage(buffDistinguish_);
    //�������
    if (rect_.size.width > 0) {
        if (_findR_Flag) {
            angleFactory_->setBuffSolverStruct(sameTargetFlag_, _findR_Flag, R_2D_Center_, rect_, buffBias_,stm32CmdData.shootSpeed);
            angleFactory_->calculateFinalResult(rect_, armorType_, distinguishMode_, carType_, stm32CmdData.pitchData.float_temp, stm32CmdData.yawData.float_temp,stm32CmdData.shootSpeed,sameTargetFlag_,targetWidth);
            _tx2Command.captureCmd = angleFactory_->getCaptureFlag();
            buffPoint_ = angleFactory_->getPronosisCoordinate();
        } else {
            _tx2Command.captureCmd = false;
            buffPoint_ = cv::Point2f(0.0f, 0.0f);
        }
    } else {
        _tx2Command.captureCmd = false;
        buffPoint_ = cv::Point2f(0.0f, 0.0f);
    }
}

//װ�װ�ʶ�����
void DecisionLevel::armorDistinguishProcess(const cv::Mat &src, bool topRest) {
    rect_ = armorDistinguish_.process(src, enemyColor_, carType_, isReset_, distinguishMode_, angleFactory_->get_yawAngle(), topRest);
    sameTargetFlag_ = armorDistinguish_.getSameTargetFlag();
    armorType_ = (ArmorType) armorDistinguish_.getArmorType();
    maxColor_ = armorDistinguish_.getMaxColor();
    roiSrc_ = armorDistinguish_.getRoi();
    bpImg_ = armorDistinguish_.getBpImg();
    targetWidth = armorDistinguish_.getTargetWidth();

    //����Ŀ�����ĵ㹩�Զ��ع�ʹ��
    targetCenter = rect_.center;

    //��������
    saveSampleImage(armorDistinguish_);

    //�������
    if (rect_.size.width > 0) {
        angleFactory_->calculateFinalResult(rect_, armorType_, distinguishMode_, carType_, stm32CmdData.pitchData.float_temp, stm32CmdData.yawData.float_temp, 
                                            stm32CmdData.shootSpeed, sameTargetFlag_, targetWidth, armorDistinguish_._topData.topFlag);
        _tx2Command.captureCmd = true;

    } else {
        _tx2Command.captureCmd = false;
    }
}

//��ȡʶ��ָ������
void DecisionLevel::getDistinguishData() {
#ifdef _LINUX
    if (serialData_->getCalReady()) {
        stm32CmdData = serialData_->getSTM32Cmd();
        shootSpeed_ = stm32CmdData.shootSpeed;
        enemyColor_ = stm32CmdData.enemyType;
        carType_ = stm32CmdData.carType;
        distinguishMode_ = stm32CmdData.distinguishMode;
        buffBias_ = serialData_->getBuffBias();
        if (buffBias_ != 0) serialData_->resetBuffBais();

        //���浱ǰ�İ����
        _tx2Command.feedbackCNTR.s16_temp = stm32CmdData.CNTR.s16_temp;
    } else {

        enemyColor_ = ENEMY_COLOR_MODE;
        distinguishMode_ = DISTINGUISH_MODE;
        carType_ = CAR_TYPE;
    }
    distinguishPart_ = DISTINGUISH_PART;
    //cout<<(int)distinguishMode_<<endl;
#else
    enemyColor_ = ENEMY_COLOR_MODE;
    distinguishMode_ = DISTINGUISH_MODE;
    distinguishPart_ = DISTINGUISH_PART;
    shootSpeed_ = SHOOT_SPEED;
#endif
}

//���ݴ���ˢ��
void DecisionLevel::distinguishDataFinishProc() {
    //    _tx2Command.pitchData.float_temp = stm32CmdData.pitchData.float_temp;
    //    _tx2Command.yawData.float_temp = stm32CmdData.yawData.float_temp;
    _tx2Command.sameTargetFlag = sameTargetFlag_;
    _tx2Command.distinguishTime.float_temp = (float) this->calculationFps_;

    //get stm32 order
    _tx2Command.getOrderFlag = !(stm32CmdData.distinguishMode == TX2_STOP);

    if (_tx2Command.captureCmd) {
        if (lastDistinguishMode_ == distinguishMode_) {
            isReset_ = false;
            angleFactory_->getAngleCalculateData(&_tx2Command, stm32CmdData);
        } else {
            isReset_ = true;
            _tx2Command.captureCmd = false;
            _tx2Command.sameTargetFlag = false;
        }
    } else {
        angleFactory_->resetAngleCalculateData(&_tx2Command);
    }

    //С��������Flag
    if (lastDistinguishMode_ == 0) {
        _topRest = true;
    }
    else if (distinguishMode_ == 1) {
        _topRest = false;
    }

    //angleFactory_->resetAngleCalculateData(&_tx2Command);
    sendDecisionCmd();
    lastDistinguishMode_ = distinguishMode_;
}

#ifndef _LINUX

//ʹ����Ƶ����ʶ�����
void DecisionLevel::threadProcUseTestVideo() {
    getDistinguishData();
   string videoPath = "D:/Users/13330/Videos/1.avi";
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        //�����Ƶ�����������򷵻�
        LOG::info("No video");
        exit(-1);
    }
    cv::Mat frame;
    while (m_isLoop) {
        cap >> frame;
        if (!frame.empty()) {
            switch (distinguishMode_) {
                case TX2_STOP:
                    break;
                case TX2_DISTINGUISH_ARMOR:
                    armorDistinguishProcess(frame, _topRest);
                    break;
                case TX2_DISTINGUISH_BUFF:
                case TX2_DISTINGUISH_BIG_BUFF:
                    buffDistinguishProcess(frame);
                    break;
            }
            rect_.points(vertices_);
            for (int i = 0; i < 4; i++) {                                                //���Ʒ���������ͼƬ��
               cv::line(frame, vertices_[i], vertices_[(i + 1) % 4], CV_RGB(0, 255, 0), 3);
            }
            if (distinguishMode_ == 1) {
                cv::circle(frame, (vertices_[0] + vertices_[2]) / 2, 5, cv::Scalar(0, 255, 0), -1, -1, 0);
                cv::circle(frame, predictPoint_, 5, cv::Scalar(255, 0, 255), -1, -1, 0);
            } else if (distinguishMode_ == TX2_DISTINGUISH_BUFF || distinguishMode_ == TX2_DISTINGUISH_BIG_BUFF) {
                cv::circle(frame, buffPoint_, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
            }
            calReady_ = true;
            cv::imshow("frame", frame);
            int keyASCII = cv::waitKey(10);
            if (keyASCII == 27) break;                                                    //ESC�˳�
        } else {                                                                        //��Ƶ�������˳�����
            exit(0);
        }
    }
}

#endif

//��������ͷ����ʶ����
void DecisionLevel::threadProcUseActual() {
    while (m_isLoop) {
        //����Ƿ�����ͼƬ����
        if (streamData_->getCalReady()) {
            getDistinguishData();
            switch (distinguishMode_) {//TX2_DISTINGUISH_BUFF
                case TX2_STOP:
                    if (!CurveData::isEmpty()) {
                        CurveData::write();
                    }
                    break;
                case TX2_DISTINGUISH_ARMOR:
                    armorDistinguishProcess(streamData_->getMatImage(), _topRest);
                case TX2_DISTINGUISH_BUFF:
                case TX2_DISTINGUISH_BIG_BUFF:
                    buffDistinguishProcess(streamData_->getMatImage());
                    break;
            }
            rect_.points(vertices_);
            calReady_ = true;                                                        //������׼�����
            calFps();                                                               //����FPS
            streamData_->setCalReady(false);                                            //����ͼ���־λ
            distinguishDataFinishProc();
        }
    }
}

//���߲����
void DecisionLevel::threadProc() {
#if TEST_VIDEO
    threadProcUseTestVideo();
#else
    threadProcUseActual();
#endif

}

