//#define USE_KCF      //使用kcf
#include "tool/Conf.h"
#include <cmath>
#include <vector>

#ifdef USE_KCF
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#endif

#include <iostream>
#include "armor/bpPredict/net.h"

#define MakeSafeRect(rect, max_size) {if (Util::makeRectSafe(rect, max_size) == false) continue;}
#define OTHER_TARGET_COUNT_LIMIT 7
#define FIFO_LEN    11

class ArmorDistinguish {
private:

    DistinguishMode _distinguishMode;

	struct ArmorRectList {
		std::vector<cv::RotatedRect> partLightBars[2];
		std::vector<cv::RotatedRect> armorRect;
	};

    struct ArmorStruct {
        cv::RotatedRect partLightBars[2];
        cv::RotatedRect armorRect;
        float lightAngle;
        bool armorType = false;
        int carId = 0;
    };
    struct ArmorRectHistoricalDataList {
        std::vector<float> area;
        std::vector<float> lengthWidthRatio;
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> areaChangeRate;
        std::vector<float> areaChangeRateChangeRate;
        std::vector<float> lenWidChangeRate;
        std::vector<float> xChangeRate;
        std::vector<float> yChangeRate;
        std::vector<float> areaChangeRateChangeRateVarianceList;
        std::vector<float> lenWidChangeRateVarianceList;
        std::vector<float> xChangeRateVarianceList;
        std::vector<float> yChangeRateVarianceList;
    };

	struct ArmorWeightCalculateCoefficient {
		float sameTargetThreshold = 0.08f;
		float areaVarianceCoefficient = 5.0f;
		float xVarianceCoefficient = 16.7f;
		float yVarianceCoefficient = 16.7f;
		float lenWidVarianceCoefficient = 0.3f;
	};
public:
    ArmorDistinguish();
    ~ArmorDistinguish() {}

	//装甲板识别过程
	cv::RotatedRect process(const cv::Mat& src, EnemyColor enemyColor, CarType carType, bool isReset,DistinguishMode distinguishMode, float yawAngle, bool topRest);

    struct SampleDataStruct getSampleData() {
        return _sampleData;
    }

    void setSampleCaptureState(bool state) {
        _sampleCaptureState = state;
    }

    bool getSampleCaptureState() const {
        return _sampleCaptureState;
    }

	bool getSameTargetFlag() const {
		return _sameTargetFlag;
	}

	bool getArmorType() const {
		return _armorType;
	}

    cv::Mat getMaxColor() {
        return _maxColor;
    }

    cv::Mat getRoi(){
        return _src;
    }

    cv::Mat getBpImg(){
        return _bpImg;
    }

    int getTargetWidth() {
        return _resultRect.size.width;
    }

    struct topData {
        float maxAngle = 0.0;
        float minAngle = 0.0;
        bool topFlag = false;
    }_topData;

private:
    /**
    * 对图片进行ROI和二值化
    * @param src 输入图片
    * @param enemyColor 敌方颜色
    */
    void imagePreprocess(const cv::Mat &src, EnemyColor enemyColor);

    /**
     * 找灯光轮廓
     * @param rects 疑似矩形
     */
    void findLightBarContour();
	/**
	 * 对灯条进行过滤
	 * @param inputRects 输入的疑似矩形
	 * @param armorStructs 装甲板结构体
	 */
	void lightBarFilter();

	/**
	 * 获得短边中点
	 * @param rect 输入矩形
	 * @param shortCenter 短边中点
	 */
	void getShortCenter(cv::RotatedRect rect, cv::Point2f* shortCenter);

	/**
	 * 计算两个矩形偏移角度
	 * @param rect1 矩形1
	 * @param rect2 矩形2
	 * @param shortCenter 短边中点数组
	 * @return
	 */
	float calSkewingAngle(cv::RotatedRect rect1, cv::RotatedRect rect2, cv::Point2f* shortCenter);

	/**
	 * 通过形态学和ml检测是否为装甲板
	 * @param armorStructs 装甲板结构体动态数组
	 */
	void armorChooseTarget(EnemyColor enemyColor);

    /**
     * 区分装甲板类型
     * @param armorStructs 装甲板结构体动态数组
     */
    void distinguishArmorType(ArmorStruct& armorStructs);

    /**
     * 找深度信息最小的装甲板;armorType(0是小，1是大)
     * @param armorStructs 装甲板结构体动态数组
     * @param armorType 装甲板类型
     * @param resultRect 最终矩形
     */
    void findNearestArmor(std::vector<ArmorStruct>& armorStructs, cv::RotatedRect& resultRect);

	/**
	 * 根据权重找装甲板
	 * @param armorStructs 装甲板结构体动态数组
	 * @param resultRect 最终矩形
	 * @param armorRectData 装甲板历史数据
	 * @param maxWeightNum 最大权重下标
	 */
    void findArmorByWeight(std::vector<ArmorStruct>& armorStructs, cv::RotatedRect& resultRect, ArmorRectHistoricalDataList& armorRectData);

    /**
     * 对ID进行攻击序列排序
     * @param armorStructs 装甲板结构体动态数组
     * @return 重新排列后的装甲板结构体动态数组
     */
    std::vector<ArmorStruct> getAttackOrder(std::vector<ArmorStruct> &armorStructs);

	/**
	 * 得到最终打击矩形
	 * @param armorStructs 装甲板结构体动态数组
	 * @return 最终打击矩形
	 */
	cv::RotatedRect getResult();
	
	/**
	 * 装甲板协方差权重计算
	 * @param data 装甲板矩形历史数据
	 * @param i 下标
	 * @return 权重
	 */
	float armorVarianceWeightCalculate(struct ArmorRectHistoricalDataList& data, size_t i);

	/**
	 * 找到权重最大的装甲板
	 * @param data 装甲板矩形历史数据
	 * @param nowArmorRectData 装甲板矩形现在数据
	 * @param weight 权重
	 * @return 权重最大装甲板下标
	 */
	int findVarianceMaxWeight(struct ArmorRectHistoricalDataList& data, struct ArmorRectHistoricalDataList& nowArmorRectData, float& weight);

	/**
	 * 装甲板方差数据装填
	 * @param lastArmorRectData 上次的装甲板数据
	 * @param nowArmorRectData 这次的装甲板数据
	 * @param i 下标
	 */
	void armorRectListVarianceDataCalculate(struct ArmorRectHistoricalDataList& lastArmorRectData, struct ArmorRectHistoricalDataList& nowArmorRectData, size_t i);

	/**
	 * 历史装甲板偏差数据装填
	 * @param lastArmorRectData 上次的装甲板数据
	 * @param nowArmorRectData 这次的装甲板数据
	 * @param i 下标
	 * @param mode 模式
	 */
	void armorRectListBiasDataIndex(struct ArmorRectHistoricalDataList& lastArmorRectData, struct ArmorRectHistoricalDataList& nowArmorRectData, uint8_t i, uint8_t mode);

	/**
	 * 历史装甲板基本数据装填
	 * @param armorRectData 装甲板数据
	 * @param rect 最终矩形
	 */
	void armorRectListBaseDataIndex(struct ArmorRectHistoricalDataList& armorRectData, cv::RotatedRect& rect);

	/**
	 * 装甲板历史数据重置
	 * @param armorRectData 装甲板数据
	 */
	void armorRectListDataReset(struct ArmorRectHistoricalDataList& armorRectData);

    /**
     * 移除误识别装甲板
     * @param armorStructs 装甲板结构体动态数组
     * @return 移除后的装甲板结构体动态数组
     */
    void removeWrongArmor();

    /**
     * 对inputMat进行HOG特征提取并预测装甲板ID
     * @param inputMat 待预测二值图
     * @return 预测结果{"0","hero","engineer","infantry3","infantry4","5","base and skirmish","sentinel"} 0与5为空弃数
     */
    int armorHogSvm(cv::Mat &inputMat);

    /**
     * 对目标灯条颜色判断
     * @param enemyColor 敌方颜色
     * @param blobPos1 灯条一
     * @param blobPos2 灯条二
     * @return true正确，false错误
     */
    bool getBlobColor(EnemyColor enemyColor, const cv::RotatedRect &blobPos1, const cv::RotatedRect &blobPos2);

    /**
     * 判断装甲板内部有无灯条
     * @param r1 装甲板矩形
     * @param r2 灯条
     * @return true内部含有灯条，false内部不含灯条
     */
    bool embeddedRectJudge(cv::RotatedRect r1, cv::RotatedRect r2);

    /**
     *小陀螺识别
     */
    void topProcess(float yawAngle);

    cv::Ptr<cv::ml::SVM> _svmHog;

    cv::Point2f _vertices[4];
    cv::Point2f _lastTargetCenter;

    int _lastCarID = 0;
    int _nowCarID = 0;
    bool _armorType = 0;
    bool _firstEnterFlag = false;
    bool _sampleCaptureState = false;
    bool _sameTargetFlag = false;
    bool _bigArmorFlag = false;                    //false是empty(),true是存在

    bool _lastsameTargetFlag = false;
    bool _topStatusFlag = false;
    bool _lastTopStatusFlag = false;
    bool _losesameFlag = false;
    bool _findsameFlag = false;
    bool _clockWiseTop = false;
    bool _antiClockWiseTop = false;
    int _cycleCount = 0;
    int Clockwise = 0;								//顺时针旋转
    int antiClockwise = 0;							//逆时针旋转
    int _diffCount = 0;
    int _trackCount = 0;
    int _untrackCount = 0;
    float _lastAngle = 0.0;
    double _oneCycletime = 0;
    double _topTime;

    int maxWeightNum;
    int _lost_cnt = 0;
    float _maxWeightValue = 0.0f;
    double _lastTime = 0.0;
    double _timeBias = 0.0;

	cv::RotatedRect _resLast;
    cv::RotatedRect _leftLightBar, _rightLightBar;
	cv::RotatedRect _resultRect;

    cv::Rect _restoreRect;
    cv::Rect leftLight;
    cv::Rect rightLight;

    cv::Mat _src;
    cv::Mat _graySrc;
    cv::Mat _separationSrc;        //色彩分离后的图像
    cv::Mat _separationSrcWhite;
    cv::Mat _separationSrcGreen;
    cv::Mat _maxColor;            //最终二值化图像
    cv::Mat _purpleSrc;
    cv::Mat _standard_2_t;
    cv::Mat _standard_2_f;
    cv::Mat _bpImg;

    std::vector<cv::Mat> splitSrc;
    std::vector<cv::RotatedRect> possibleRects;
    std::vector<ArmorStruct> armorStructs;                     //轮廓
    std::vector<std::vector<cv::Point2i>> allContours;                                //轮廓
    std::vector<cv::Vec4i> hierarchy;
    std::vector<ArmorStruct> orderArmorStructs;

    //cv::dnn::Net net;
    DistinguishMode _armorMode;
    CarType _carType = INFANTRY;
	ArmorPara _para = ArmorParaFactory::getArmorPara();
	struct SampleDataStruct _sampleData;
	ArmorWeightCalculateCoefficient _armorWeightCalculateCoefficient;
	ArmorRectHistoricalDataList _armorRectHistoricalData;
	uint32_t _otherTargetCount = 0;
	bp::Net _net;
#ifdef USE_KCF
    //追踪器
    void armorTrackerInit(const cv::Mat& src, EnemyColor enemyColor);
    cv::Ptr<cv::TrackerKCF> _tracker = cv::TrackerKCF::create();
    bool _startTracker = false;
    cv::TrackerKCF::Params _params;
    bool _isTrackerFind = false;
    cv::Rect2d _trackerRect = cv::Rect2d(0, 0, 0, 0);
    cv::Rect2d _searchRect = cv::Rect2d(0, 0, 0, 0);
    bool _isReulstFind = false;
    int _trackerLost = 0;
    bool _trackerSuccess = false;
    bool _isTrackerStart = false;
    int _imgWidth = 1920;
    int _imgHeight = 1200;
    cv::Size _size;
#endif
};
