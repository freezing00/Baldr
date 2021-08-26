#ifndef _BUFF_DISTINGUISH_H_
#define _BUFF_DISTINGUISH_H_


#include "tool/Conf.h"

class BuffDistinguish {
public:

    BuffDistinguish();

    cv::RotatedRect process(const cv::Mat &src, OwnColor ownColor);

    bool getSameTargetFlag() const {
        return _sameTargetFlag;
    }

    cv::Point2f getR_2D_Center() {
        return _R_2D_Center;
    }

    bool getCaptureR_Flag() const {
        return _captureR_Flag;
    }

    bool getFindRectFlag() const {
        return _findRectFlag;
    }

    struct SampleDataStruct getSampleData() {
        return _sampleData;
    }

    struct SampleDataStruct getCircleSampleData() {
        return _circleSampleData;
    }

    void setSampleCaptureState(bool state) {
        _sampleCaptureState = state;
    }

    bool getSampleCaptureState() const {
        return _sampleCaptureState;
    }

private:

    struct BuffToBeat {
        cv::RotatedRect cantileverRect;
        cv::RotatedRect resultRect;
        cv::RotatedRect armorRect;
    };
    struct BuffAlBeat {
        cv::RotatedRect cantileverRect;
        cv::RotatedRect armorRect;
        int embeddedNum;
        float cantileverContoursAreas;
    };

    /**
     * ��ͼƬ����ROI�Ͷ�ֵ������
     * @param src ����ͼƬ
     * @param ownColor ������ɫ
     */
    void imagePreprocess(const cv::Mat &src, OwnColor ownColor);

    /**
     * �ҵƹ�����
     * @param rects ���ƾ���
     */
    void findLightBarContour(std::vector<cv::RotatedRect> &rects, std::vector<float> &areas);

    /**
     * ��Բ�ľ���
     * @param circleCenterRect Բ�ľ���
     */
    void findCircleCenter(cv::RotatedRect &circleCenterRect);

    /**
     * Բ��SVM
     * @param inputRect ����ľ���
     * @return true-SVM��ΪԲ�ģ�false-SVM��Ϊ����Բ��
     */
    bool circleCenterSVM(cv::RotatedRect &inputRect);

    /**
     * �Ӿ������ҵ�װ�װ������
     * @param inputRects �������ƾ���
     * @param buffToBeat ��������μ���
     */
    void findCantilever(std::vector<cv::RotatedRect> &inputRects, BuffToBeat &buffToBeat, vector<float> &inputAreas);

    /**
     * �ҵ����п��ܵ����ۺ�װ�װ�
     * @param rects �������ƾ���
     * @param cantileverRects ���۾��μ�
     * @param armorRects װ�װ���μ�
     */
    void findPossibleRects(vector<cv::RotatedRect> rects, vector<float> &areas,
                           vector<cv::RotatedRect> &cantileverRects, vector<cv::RotatedRect> &armorRects);

    /**
     * ����ҶƬ�ṹ��Ĺ���
     * @param cantileverRects ���۾��μ�
     * @param armorRects װ�װ���μ�
     * @param realBuffToBeat �����ҶƬ�ṹ��
     */
    void makeBuffStruct(std::vector<cv::RotatedRect> cantileverRects, std::vector<cv::RotatedRect> armorRects, BuffToBeat &realBuffToBeat);

    /**
     * ���ִ�������Ѵ��ҶƬ�ṹ��
     * @param buffToBeat �����ҶƬ�ṹ��
     * @param buffAlBeat �Ѵ��ҶƬ�ṹ��
     * @param alBuffStruct ��ϵ�ҶƬ�ṹ��
     */
    void distinguishBuffStruct(BuffToBeat &buffToBeat, std::vector<BuffAlBeat> &buffAlBeat, std::vector<BuffAlBeat> &alBuffStruct);


    cv::Mat _src;
    cv::Mat _maxColor;
    cv::Mat _graySrc;
    cv::Mat _separationSrcGreen;
    cv::Mat _separationSrc;        //ɫ�ʷ�����ͼ��

    std::vector<cv::Mat> _splitSrc;

    cv::Point2f _R_2D_Center = cv::Point2f();
    cv::Point2f _last_R_2D_Center = cv::Point2f();
    cv::Point _lu = cv::Point(0, 0);
    cv::Point _rd = cv::Point(0, 0);

    cv::RotatedRect _resLast;
    cv::RotatedRect _circleCenterRect;

    std::vector<cv::RotatedRect> _suspicion_R_Rects;

    int _lost_cnt = 0;
    int _find_cnt = 0;

    float _circleCenterLong;
    float _circleCenterArea;

    bool _findRectFlag = false;
    bool _sameTargetFlag = false;
    bool _captureR_Flag = false;
    bool _sampleCaptureState = false;

    struct SampleDataStruct _sampleData;
    struct SampleDataStruct _circleSampleData;

    cv::Mat _testMat = cv::Mat();

    std::vector<BuffAlBeat> _buffAlBeats;
    cv::Rect _restoreRect;
    BuffPara _para = BuffParaFactory::getPara();
    ParaCircle _paraCircle = BuffParaFactory::getParaCircle();
    cv::Ptr<cv::ml::SVM> _circleCenterSvm;
};

#endif
