#pragma once

#include "Eigen/Dense"
#include "opencv2/opencv.hpp" 
#include <vector>
#include <fstream>
#include <iostream>
#include "tool/RMDefine.h"

//保存曲线文件路径
#define CURVE_DATA_PATH "../visionData/curve/"

//最大间隔时间300ms
#define MAX_TIME_BIAS 300.0f

//速度计算存储数据长度，最好为4的倍数 
#define V_CAL_MAX_VAL (4*8)
//所允许的最大坐标值	允许极限距离16m
#define MAX_COORDINATE_XZ	16.0f
#define MAX_COORDINATE_Y	2.0f
//坐标最大跳变
#define FILTER_COORDINATE_MAX 0.5f

//所允许的最大速度
#define MAX_VELOCITY_XZ 7.0f
#define MAX_VELOCITY_Y  1.0f
//速度最大跳变
#define FILTER_V_MAX 1.0f
//加速度最大跳变
#define FILTER_A_MAX 5.0f

//预判速度限制
#define PREDICT_MAX_SPEED	7.0f
#define PREDICT_MIN_SPEED	0.1f
//预判加速度限制
#define PREDICT_MAX_ACC		20.0f
#define PREDICT_MIN_ACC		0.5f

//编程依据:
//https://zhuanlan.zhihu.com/p/45238681

//卡尔曼滤波的通俗理解（强烈推荐）:
//https://zhuanlan.zhihu.com/p/39912633?utm_source=qq&utm_medium=social&utm_oi=946351736747794432


class KalmanFilter {
public:
	KalmanFilter() {
		_initFlag = false;
	}

	~KalmanFilter() {}

	//初始化卡尔曼参数（状态转移矩阵中含有时间间隔，需要随迭代更新）
	void InitParam(Eigen::MatrixXd inputP, Eigen::MatrixXd inputQ, Eigen::MatrixXd inputH, Eigen::MatrixXd inputR) {
		SetP(inputP);

		SetQ(inputQ);

		SetH(inputH);

		SetR(inputR);
	}

    void setLimit(cv::Point3f maxLimit, cv::Point3f jumpLimit) {
        _maxLimit = maxLimit;
        _jumpLimit = jumpLimit;
    }

	Eigen::VectorXd GetX() {
		return _x;
	}

	bool isInitialized() {
		return _initFlag;
	}

	void Initialization(Eigen::VectorXd inputX) {
		_x = inputX;
		_initFlag = true;
	}

	void InitializationX(Eigen::VectorXd inputX) {
		_x = inputX;
	}

	void SetF(Eigen::MatrixXd inputF) {
		_f = inputF;
	}

	void SetP(Eigen::MatrixXd inputP) {
		_p = inputP;
	}

	void SetQ(Eigen::MatrixXd inputQ) {
		_q = inputQ;
	}

	void SetH(Eigen::MatrixXd inputH) {
		_h = inputH;
	}

	void SetR(Eigen::MatrixXd inputR) {
		_r = inputR;
	}

	void setInitFalse() {
		_initFlag = false;
	}

	/**
	 * 卡尔曼滤波
	 */
	void Prediction();

	/**
	 * 更新测量值
	 * @param z 测量值
	 */
	void MeasurementUpdate(const Eigen::VectorXd& z);

	/**
	 * 卡尔曼滤波主函数
	 * @param originData 原始数据
	 */
	cv::Point3f KPredictionRun(cv::Point3f originData);

	/**
	 * 剔除异常数据
	 * @param originData 原始数据
	 */
	cv::Point3f wrongDataKiller(cv::Point3f originData);
	/**
	 * 用于调用预判与观测函数
	 */
	cv::Point3f KFilter(cv::Point3f coordinate);

	/**
	* 获取当前坐标对应的时间（ms）
	*/
	float getKFtime() {
		return _nowTime_ms;
	}

private:

	bool _initFlag = false;													//初始化标志位
    int _badCntC = 0;
	float _deltaTime_ms = 0.0f, _nowTime_ms = 0.0f, _lastTime_ms = 0.0f;		//单位ms
	cv::Point3f _lastOriginData;
    cv::Point3f _maxLimit;
    cv::Point3f _jumpLimit;
	Eigen::VectorXd _x;		//状态矩阵包含 位置、速度
	Eigen::MatrixXd _f;		//状态转移矩阵
	Eigen::MatrixXd _p;		//状态协方差矩阵
	Eigen::MatrixXd _q;		//过程噪声矩阵
	Eigen::MatrixXd _h;		//测量矩阵
	Eigen::MatrixXd _r;		//测量噪声矩阵
};

//曲线数据
class CurveData {
private:
	static CurveData& instance() {
		static CurveData curveData;
		return curveData;
	}

    const bool CURVE_SWITCH = true;				//曲线绘制开关
	std::vector<float> _nowTime_ms;					//存储坐标点对应时间
	std::vector<cv::Point3f> _velocityOrigin;		//存储坐标点速度
	std::vector<cv::Point3f> _velocityFilter;		//存储坐标点速度
	std::vector<cv::Point3f> accOrigin;			//存储加速度数据
	std::vector<cv::Point3f> _coordinateOrigin;		//存储原始坐标点数据
	std::vector<cv::Point3f> _coordinatePredict;	//存储预判坐标点数据
	std::vector<cv::Point3f> _coordinateFilter;		//存储滤波坐标点数据
	cv::Point3f _coordinateNoBias;		//无补偿量时的预判坐标点数据
	std::vector<cv::Point2f> _angleNoBias;
    std::vector<cv::Point2f> _angleRef;
    std::vector<cv::Point2f> _angleFbd;
    std::vector<float>  _angleTime;
	int _writeCnt = 0;
	const int MAX_WRITE_CNT = 300;
    bool _isEmpty = true;

public:

	/**
	 * 保存坐标点
	 * @param point	水平坐标点（不要被xyz的顺序迷惑）
	 */
	static void saveTime(float);
	static void saveVelocity(cv::Point3f origin, cv::Point3f Filter);
	static void saveAcc(cv::Point3f origin);
	static void saveCoordinate( cv::Point3f origin, cv::Point3f filter, cv::Point3f prediction);
	static void saveNoBiasCoordinate(cv::Point3f coordinate);
	static cv::Point3f getNoBiasCoordinate();
    static void saveAngle(cv::Point2f angleNoBias, cv::Point2f angleRef, cv::Point2f angleFbd);

	/**
	 * 清空坐标
	 */
	static void clear();

	/**
	* 写入曲线文件
	* 每一行为一组数据，不同轴上的数据用空格隔开
	*/
	static void write();

	/**
	* 数据是否为空
	*/
	static bool isEmpty();

	/**
	* 是否启用曲线绘制
	*/
	static bool isEnable();

};

class Prediction {
public:
	Prediction() {
		Eigen::MatrixXd VinputP(6, 6);		//初始化状态协方差矩阵
		VinputP <<	1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
					0.0, 0.0, 0.0, 1.0, 0.0, 1.0;

		Eigen::MatrixXd VinputQ(6, 6);		//初始化过程噪声矩阵
        VinputQ <<	7.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 7.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 200.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 5.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 200.0;

		Eigen::MatrixXd VinputH(3, 6);		//初始化观测矩阵
		VinputH <<	1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		Eigen::MatrixXd VinputR(3, 3);		//初始化观测噪声矩阵
        VinputR <<	1000.0, 0.0, 0.0,
                    0.0, 1000.0, 0.0,
                    0.0, 0.0, 1000.0;

		_velocityFilter.InitParam(VinputP, VinputQ, VinputH, VinputR);
        _velocityFilter.setLimit(cv::Point3f(MAX_VELOCITY_XZ, MAX_VELOCITY_Y, MAX_VELOCITY_XZ), cv::Point3f(FILTER_V_MAX, FILTER_V_MAX, FILTER_V_MAX));

        Eigen::MatrixXd PinputP(6, 6);		//初始化状态协方差矩阵
		PinputP <<	1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
					0.0, 0.0, 0.0, 1.0, 0.0, 1.0;

        Eigen::MatrixXd PinputQ(6, 6);		//初始化过程噪声矩阵
		PinputQ <<	10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 10.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 50.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 50.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 50.0;

		Eigen::MatrixXd PinputH(3, 6);		//初始化观测矩阵
		PinputH <<	1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		Eigen::MatrixXd PinputR(3, 3);		//初始化观测噪声矩阵
		PinputR <<	1000.0, 0.0, 0.0,
					0.0, 1000.0, 0.0,
					0.0, 0.0, 1000.0;

		_predictionFilter.InitParam(PinputP, PinputQ, PinputH, PinputR);
        _predictionFilter.setLimit(cv::Point3f(MAX_COORDINATE_XZ, MAX_COORDINATE_Y, MAX_COORDINATE_XZ), cv::Point3f(FILTER_COORDINATE_MAX, FILTER_COORDINATE_MAX, FILTER_COORDINATE_MAX));
    }

	~Prediction() {};

	/**
	 * pitch轴角度计算
	 * @param coordinate	固有坐标系下的坐标 单位为mm
	 * @param shootSpeed	弹丸初速
	 */
	float angleCalculation(cv::Point3f coordinate, float shootSpeed);
	/**
	 * 带空气阻力近似的pitch轴角度计算
	 * @param coordinate	固有坐标系下的坐标 单位为mm
	 * @param shootSpeed	弹丸初速
	 */
	float airResistCalculation(cv::Point3f coordinate, float shootSpeed);
	/**
	 * 预判函数
	 * @param coordinate	坐标数据 单位mm
	 * @param shootSpeed	弹丸初速
	 * @param coordinateV	坐标轴上的速度
	 * 只预判XOZ平面上的坐标
	 */
	cv::Point3f prediction(cv::Point3f coordinate, float time, float shootSpeed, CarType carType);
	/**
	 * 获取时间间隔
	 * @param time 时间 单位ms
	 */
	void getDeltaTime(float time);
	/**
	 * 获得速度与加速度
	 * @param coordinate 坐标数据 单位mm
	 */
	void dataCal(cv::Point3f coordinate);
	/**
	 * “逐差法”
	 * @param data 参与运算的数据
	 * @param result 输出的结果
	 */
	void subMethod(float data[][3], cv::Point3f& result);

	cv::Point3f Iteration(cv::Point3f coordinate_m, float shootSpeed, CarType carType);

private:

    float _timeBias = 0.1f;												//考虑到现实中的各种延迟误差，加入补偿量
	float _deltaTime_ms = 0, _nowTime_ms = 0.0f, _lastTime_ms = 0.0f;

	KalmanFilter _velocityFilter;
	KalmanFilter _predictionFilter;

	bool _initFlag = false;											//初始化标志位
	int _dataCnt = 0;												//数据计数
    float _data_m[V_CAL_MAX_VAL][3] = { 0.0f };					//坐标数据，单位mm
	float _velo_s[V_CAL_MAX_VAL][3] = { 0.0f };						//速度数据，单位m/s
	cv::Point3f _V = cv::Point3f(0.0f, 0.0f, 0.0f);					//x y z轴坐标速度 （y目前不使用，恒为0）
	cv::Point3f _lastV = cv::Point3f(0.0f, 0.0f, 0.0f);
	cv::Point3f _A = cv::Point3f(0.0f, 0.0f, 0.0f);					//x y z轴坐标加速度（y目前不使用，恒为0）
	cv::Point3f _lastA = cv::Point3f(0.0f, 0.0f, 0.0f);
	float _deltaTime_s[V_CAL_MAX_VAL - 1] = { 0.0f };				//时间间隔，单位s
};

#pragma once
