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
    uchar distinguishPart_ = 0;                         //ʶ�𲿷֣�0Ϊ���ۣ�1ΪԲ�ģ�ǰ��Ϊʶ��ģʽΪ�����
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
     * �߳�ִ����
     */
    void threadProc();

    /**
     * ������Ƶ�߳�ִ����
     */
    void threadProcUseTestVideo();

    /**
     * ��ҵ����߳�ִ����
     */
    void threadProcUseActual();

    /**
     * ���ʶ������
     * @param src ͼƬ
     */
    void buffDistinguishProcess(const cv::Mat &src);

    /**
     * װ�װ�ʶ������
     * @param src ͼƬ
     */
    void armorDistinguishProcess(const cv::Mat &src, bool topRest);

    /**
     * �����λ������
     */
    void getDistinguishData();

    /**
     * ���һ��ʶ�����̵��ô˺���
     */
    void distinguishDataFinishProc();

    /**
     * ��32����ָ��
     */
    void sendDecisionCmd();

    /**
     * ��������ͼƬ
     */
    void saveSampleImage(ArmorDistinguish &armorDistinguish);

    /**
     * ��������ͼƬ
     */
    void buffSaveSampleImage(BuffDistinguish &buffDistinguish);

    /**
    * �����߳������ٶ�
    */
    void calFps();
};

typedef Dahua::Memory::TSharedPtr<DecisionLevel> DecisionLevelPtr;
#endif
