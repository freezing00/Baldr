//#define USE_KCF      //ʹ��kcf
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

	//װ�װ�ʶ�����
	cv::RotatedRect process(const cv::Mat& src, EnemyColor enemyColor, CarType carType, bool isReset,DistinguishMode distinguishMode);

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
private:
    /**
    * ��ͼƬ����ROI�Ͷ�ֵ��
    * @param src ����ͼƬ
    * @param enemyColor �з���ɫ
    */
    void imagePreprocess(const cv::Mat &src, EnemyColor enemyColor);

    /**
     * �ҵƹ�����
     * @param rects ���ƾ���
     */
    void findLightBarContour();
	/**
	 * �Ե������й���
	 * @param inputRects ��������ƾ���
	 * @param armorStructs װ�װ�ṹ��
	 */
	void lightBarFilter();

	/**
	 * ��ö̱��е�
	 * @param rect �������
	 * @param shortCenter �̱��е�
	 */
	void getShortCenter(cv::RotatedRect rect, cv::Point2f* shortCenter);

	/**
	 * ������������ƫ�ƽǶ�
	 * @param rect1 ����1
	 * @param rect2 ����2
	 * @param shortCenter �̱��е�����
	 * @return
	 */
	float calSkewingAngle(cv::RotatedRect rect1, cv::RotatedRect rect2, cv::Point2f* shortCenter);

	/**
	 * ͨ����̬ѧ��ml����Ƿ�Ϊװ�װ�
	 * @param armorStructs װ�װ�ṹ�嶯̬����
	 */
	void armorChooseTarget(EnemyColor enemyColor);

    /**
     * ����װ�װ�����
     * @param armorStructs װ�װ�ṹ�嶯̬����
     */
    void distinguishArmorType(ArmorStruct& armorStructs);

    /**
     * �������Ϣ��С��װ�װ�;armorType(0��С��1�Ǵ�)
     * @param armorStructs װ�װ�ṹ�嶯̬����
     * @param armorType װ�װ�����
     * @param resultRect ���վ���
     */
    void findNearestArmor(std::vector<ArmorStruct>& armorStructs, cv::RotatedRect& resultRect);

	/**
	 * ����Ȩ����װ�װ�
	 * @param armorStructs װ�װ�ṹ�嶯̬����
	 * @param resultRect ���վ���
	 * @param armorRectData װ�װ���ʷ����
	 * @param maxWeightNum ���Ȩ���±�
	 */
    void findArmorByWeight(std::vector<ArmorStruct>& armorStructs, cv::RotatedRect& resultRect, ArmorRectHistoricalDataList& armorRectData);

    /**
     * ��ID���й�����������
     * @param armorStructs װ�װ�ṹ�嶯̬����
     * @return �������к��װ�װ�ṹ�嶯̬����
     */
    std::vector<ArmorStruct> getAttackOrder(std::vector<ArmorStruct> &armorStructs);

	/**
	 * �õ����մ������
	 * @param armorStructs װ�װ�ṹ�嶯̬����
	 * @return ���մ������
	 */
	cv::RotatedRect getResult();
	
	/**
	 * װ�װ�Э����Ȩ�ؼ���
	 * @param data װ�װ������ʷ����
	 * @param i �±�
	 * @return Ȩ��
	 */
	float armorVarianceWeightCalculate(struct ArmorRectHistoricalDataList& data, size_t i);

	/**
	 * �ҵ�Ȩ������װ�װ�
	 * @param data װ�װ������ʷ����
	 * @param nowArmorRectData װ�װ������������
	 * @param weight Ȩ��
	 * @return Ȩ�����װ�װ��±�
	 */
	int findVarianceMaxWeight(struct ArmorRectHistoricalDataList& data, struct ArmorRectHistoricalDataList& nowArmorRectData, float& weight);

	/**
	 * װ�װ巽������װ��
	 * @param lastArmorRectData �ϴε�װ�װ�����
	 * @param nowArmorRectData ��ε�װ�װ�����
	 * @param i �±�
	 */
	void armorRectListVarianceDataCalculate(struct ArmorRectHistoricalDataList& lastArmorRectData, struct ArmorRectHistoricalDataList& nowArmorRectData, size_t i);

	/**
	 * ��ʷװ�װ�ƫ������װ��
	 * @param lastArmorRectData �ϴε�װ�װ�����
	 * @param nowArmorRectData ��ε�װ�װ�����
	 * @param i �±�
	 * @param mode ģʽ
	 */
	void armorRectListBiasDataIndex(struct ArmorRectHistoricalDataList& lastArmorRectData, struct ArmorRectHistoricalDataList& nowArmorRectData, uint8_t i, uint8_t mode);

	/**
	 * ��ʷװ�װ��������װ��
	 * @param armorRectData װ�װ�����
	 * @param rect ���վ���
	 */
	void armorRectListBaseDataIndex(struct ArmorRectHistoricalDataList& armorRectData, cv::RotatedRect& rect);

	/**
	 * װ�װ���ʷ��������
	 * @param armorRectData װ�װ�����
	 */
	void armorRectListDataReset(struct ArmorRectHistoricalDataList& armorRectData);

    /**
     * �Ƴ���ʶ��װ�װ�
     * @param armorStructs װ�װ�ṹ�嶯̬����
     * @return �Ƴ����װ�װ�ṹ�嶯̬����
     */
    void removeWrongArmor();

    /**
     * ��inputMat����HOG������ȡ��Ԥ��װ�װ�ID
     * @param inputMat ��Ԥ���ֵͼ
     * @return Ԥ����{"0","hero","engineer","infantry3","infantry4","5","base and skirmish","sentinel"} 0��5Ϊ������
     */
    int armorHogSvm(cv::Mat &inputMat);

    /**
     * ��Ŀ�������ɫ�ж�
     * @param enemyColor �з���ɫ
     * @param blobPos1 ����һ
     * @param blobPos2 ������
     * @return true��ȷ��false����
     */
    bool getBlobColor(EnemyColor enemyColor, const cv::RotatedRect &blobPos1, const cv::RotatedRect &blobPos2);

    /**
     * �ж�װ�װ��ڲ����޵���
     * @param r1 װ�װ����
     * @param r2 ����
     * @return true�ڲ����е�����false�ڲ���������
     */
    bool embeddedRectJudge(cv::RotatedRect r1, cv::RotatedRect r2);

    cv::Ptr<cv::ml::SVM> _svmHog;

    cv::Point2f _vertices[4];
    cv::Point2f _lastTargetCenter;

    int _lastCarID = 0;
    int _nowCarID = 0;
    bool _armorType = 0;
    bool _firstEnterFlag = false;
    bool _sampleCaptureState = false;
    bool _sameTargetFlag = false;
    bool _bigArmorFlag = false;                    //false��empty(),true�Ǵ���

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
    cv::Mat _separationSrc;        //ɫ�ʷ�����ͼ��
    cv::Mat _separationSrcWhite;
    cv::Mat _separationSrcGreen;
    cv::Mat _maxColor;            //���ն�ֵ��ͼ��
    cv::Mat _purpleSrc;
    cv::Mat _standard_2_t;
    cv::Mat _standard_2_f;
    cv::Mat _bpImg;

    std::vector<cv::Mat> splitSrc;
    std::vector<cv::RotatedRect> possibleRects;
    std::vector<ArmorStruct> armorStructs;                     //����
    std::vector<std::vector<cv::Point2i>> allContours;                                //����
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
    //׷����
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
