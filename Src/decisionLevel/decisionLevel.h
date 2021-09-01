#ifndef _DECISION_LEVEL_H_
#define _DECISION_LEVEL_H_

#include <angle/angleFactory.h>
#include "camera/streamRetrieve.h"
#include "buff/buffTest.h"
#include "armor/armorDistinguish.h"
#include "buff/buffDistinguish.h"
#include "serial/serial.h"

#define TEST_VIDEO 0

class DecisionLevel : public Dahua::Infra::CThread {
public:
    cv::Mat DEBUGMAT;
    cv::Mat GETDEBUGMAT();
    DecisionLevel(const string &serialNumber);

    DecisionLevel(StreamRetrievePtr &StreamData, const string &serialNumber);

    DecisionLevel(StreamRetrievePtr &StreamData, SerialPtr &SerialData, const string &serialNumber);

    bool start() {
        m_isLoop = true;
        return createThread();
    }

    void stop() {
        m_isLoop = false;

    }

    cv::Mat getOutputImage() {
        return outImage_;
    }

    cv::Mat getMaxColor() {
        return maxColor_;
    }

    cv::Mat getRoi(){
        return roiSrc_;
    }

    cv::Mat getBuffBinMat(){
        return buffBinMat_;
    }


    cv::Mat getBpImg(){
        return bpImg_;
    }

    cv::Point2f getTargetCenter() const {
        return targetCenter;
    }

    cv::Size2f getTargetSize() const {
        return rect_.size;
    }

    cv::Point2f *getVertices() {
        return vertices_;
    }

    cv::Point2f *getVerticesPronosis() {
        return verticesPronosis_;
    }

    double getCalculationFps() const {
        return calculationFps_;
    }

    bool getCalReady() {
        return calReady_;
    }

    void setOutputImage(cv::Mat *src) {
        outImage_ = src->clone();
    }

    void setCalReady(bool iReady) {
        calReady_ = iReady;
    }

    bool getCaptureState() const {
        return captureState_;
    }

    DistinguishMode getDistinguishMode() const {
        return distinguishMode_;
    }

    uchar getDistinguishPart() const {
        return distinguishPart_;
    }

    struct SampleDataStruct getSampleData() {
        return sampleData_;
    }

    cv::Point2f getBuffPoint() {
        return buffPoint_;
    }

    cv::Point2f getRPoint() {
        return R_2D_Center_;
    }

    cv::Point2f getPredictPoint() {
        return predictPoint_;
    }

    cv::Point2f *getPredictRect() {
        return predictRectPoint_;
    }

    cv::Point2f *getBuffVertices() {
        return _buffVertices;
    }

    cv::Mat getBuffDrawMat(){
        return drawMat_;
    }

    Stm32CmdStruct getStm32CmdData() {
        return stm32CmdData;
    }

    DistinguishMode getDistinguishMode() {
        return distinguishMode_;
    }

    cv::Mat GETDEBUGMAT1(){
        return DEBUGMAT1;
    }

private:
    Buff _buff;
    TX2CmdStruct _tx2Command;
    Stm32CmdStruct stm32CmdData;
    SampleDataStruct sampleData_;
    SampleDataStruct circleSampleData_;

    cv::Point2f _buffVertices[4];
    cv::Point2f predictPoint_;
    cv::Point2f targetCenter;
    cv::Point2f R_2D_Center_;
    cv::Point2f buffPoint_;
    cv::Point2f vertices_[4];
    cv::Point2f verticesPronosis_[4];
    cv::Point2f predictRectPoint_[4];

    bool sameTargetFlag_ = false;
    bool captureR_Flag_ = false;
    bool _findR_Flag = false;
    bool captureState_ = false;
    bool m_isLoop;
    bool calReady_;
    double calculationFps_ = 0;
    int shootSpeed_ = 0;
    int targetWidth = 0;

    EnemyColor enemyColor_ = ENEMY_BLUE;
    DistinguishMode distinguishMode_ = TX2_STOP;
    uchar distinguishPart_ = 0;                         //识别部分，0为悬臂，1为圆心（前提为识别模式为神符）
    DistinguishMode lastDistinguishMode_ = TX2_STOP;
    CarType carType_ = INFANTRY;
    ArmorType armorType_ = ARMOR_SMALL;

    BuffBias buffBias_;
    cv::RotatedRect rect_;
    cv::RotatedRect Rrect_;
    cv::Mat outImage_;
    cv::Mat buffBinMat_;
    cv::Mat DEBUGMAT1;
    cv::Mat drawMat_;
    cv::Mat maxColor_;
    cv::Mat roiSrc_;
    cv::Mat bpImg_;

    StreamRetrievePtr streamData_;
    AngleFactory *angleFactory_;
    SerialPtr serialData_;
    ArmorDistinguish armorDistinguish_;
    BuffDistinguish buffDistinguish_;

    bool isReset_ = false;

    bool _topRest = false;

    /**
     * 线程执行体
     */
    void threadProc();

    /**
     * 测试视频线程执行体
     */
    void threadProcUseTestVideo();

    /**
     * 工业相机线程执行体
     */
    void threadProcUseActual();

    /**
     * 神符识别流程
     * @param src 图片
     */
    void buffDistinguishProcess(const cv::Mat &src);

    /**
     * 装甲板识别流程
     * @param src 图片
     */
    void armorDistinguishProcess(const cv::Mat &src, bool topRest);

    /**
     * 获得下位机命令
     */
    void getDistinguishData();

    /**
     * 完成一次识别流程调用此函数
     */
    void distinguishDataFinishProc();

    /**
     * 给32发送指令
     */
    void sendDecisionCmd();

    /**
     * 设置样本图片
     */
    void saveSampleImage(ArmorDistinguish &armorDistinguish);

    /**
     * 设置样本图片
     */
    void buffSaveSampleImage(BuffDistinguish &buffDistinguish);

    /**
    * 计算线程运行速度
    */
    void calFps();
};

typedef Dahua::Memory::TSharedPtr<DecisionLevel> DecisionLevelPtr;
#endif
