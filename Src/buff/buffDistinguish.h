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
     * 对图片进行ROI和二值化处理
     * @param src 输入图片
     * @param ownColor 己方颜色
     */
    void imagePreprocess(const cv::Mat &src, OwnColor ownColor);

    /**
     * 找灯光轮廓
     * @param rects 疑似矩形
     */
    void findLightBarContour(std::vector<cv::RotatedRect> &rects, std::vector<float> &areas);

    /**
     * 找圆心矩形
     * @param circleCenterRect 圆心矩形
     */
    void findCircleCenter(cv::RotatedRect &circleCenterRect);

    /**
     * 圆心SVM
     * @param inputRect 输入的矩形
     * @return true-SVM判为圆心，false-SVM判为不是圆心
     */
    bool circleCenterSVM(cv::RotatedRect &inputRect);

    /**
     * 从矩形中找到装甲板和悬臂
     * @param inputRects 所有疑似矩形
     * @param buffToBeat 待打击矩形集合
     */
    void findCantilever(std::vector<cv::RotatedRect> &inputRects, BuffToBeat &buffToBeat, vector<float> &inputAreas);

    /**
     * 找到所有可能的悬臂和装甲板
     * @param rects 所有疑似矩形
     * @param cantileverRects 悬臂矩形集
     * @param armorRects 装甲板矩形集
     */
    void findPossibleRects(vector<cv::RotatedRect> rects, vector<float> &areas,
                           vector<cv::RotatedRect> &cantileverRects, vector<cv::RotatedRect> &armorRects);

    /**
     * 进行叶片结构体的构建
     * @param cantileverRects 悬臂矩形集
     * @param armorRects 装甲板矩形集
     * @param realBuffToBeat 待打击叶片结构体
     */
    void makeBuffStruct(std::vector<cv::RotatedRect> cantileverRects, std::vector<cv::RotatedRect> armorRects, BuffToBeat &realBuffToBeat);

    /**
     * 区分待打击和已打击叶片结构体
     * @param buffToBeat 待打击叶片结构体
     * @param buffAlBeat 已打击叶片结构体
     * @param alBuffStruct 混合的叶片结构体
     */
    void distinguishBuffStruct(BuffToBeat &buffToBeat, std::vector<BuffAlBeat> &buffAlBeat, std::vector<BuffAlBeat> &alBuffStruct);


    cv::Mat _src;
    cv::Mat _maxColor;
    cv::Mat _graySrc;
    cv::Mat _separationSrcGreen;
    cv::Mat _separationSrc;        //色彩分离后的图像

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
