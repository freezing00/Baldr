#pragma once

#include "Eigen/Dense"
#include "opencv2/opencv.hpp" 
#include <vector>
#include <fstream>
#include <iostream>
#include "tool/RMDefine.h"

//���������ļ�·��
#define CURVE_DATA_PATH "../visionData/curve/"

//�����ʱ��300ms
#define MAX_TIME_BIAS 300.0f

//�ٶȼ���洢���ݳ��ȣ����Ϊ4�ı��� 
#define V_CAL_MAX_VAL (4*8)
//��������������ֵ	�����޾���16m
#define MAX_COORDINATE_XZ	16.0f
#define MAX_COORDINATE_Y	2.0f
//�����������
#define FILTER_COORDINATE_MAX 0.5f

//�����������ٶ�
#define MAX_VELOCITY_XZ 7.0f
#define MAX_VELOCITY_Y  1.0f
//�ٶ��������
#define FILTER_V_MAX 1.0f
//���ٶ��������
#define FILTER_A_MAX 5.0f

//Ԥ���ٶ�����
#define PREDICT_MAX_SPEED	7.0f
#define PREDICT_MIN_SPEED	0.1f
//Ԥ�м��ٶ�����
#define PREDICT_MAX_ACC		20.0f
#define PREDICT_MIN_ACC		0.5f

//�������:
//https://zhuanlan.zhihu.com/p/45238681

//�������˲���ͨ����⣨ǿ���Ƽ���:
//https://zhuanlan.zhihu.com/p/39912633?utm_source=qq&utm_medium=social&utm_oi=946351736747794432


class KalmanFilter {
public:
	KalmanFilter() {
		_initFlag = false;
	}

	~KalmanFilter() {}

	//��ʼ��������������״̬ת�ƾ����к���ʱ��������Ҫ��������£�
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
	 * �������˲�
	 */
	void Prediction();

	/**
	 * ���²���ֵ
	 * @param z ����ֵ
	 */
	void MeasurementUpdate(const Eigen::VectorXd& z);

	/**
	 * �������˲�������
	 * @param originData ԭʼ����
	 */
	cv::Point3f KPredictionRun(cv::Point3f originData);

	/**
	 * �޳��쳣����
	 * @param originData ԭʼ����
	 */
	cv::Point3f wrongDataKiller(cv::Point3f originData);
	/**
	 * ���ڵ���Ԥ����۲⺯��
	 */
	cv::Point3f KFilter(cv::Point3f coordinate);

	/**
	* ��ȡ��ǰ�����Ӧ��ʱ�䣨ms��
	*/
	float getKFtime() {
		return _nowTime_ms;
	}

private:

	bool _initFlag = false;													//��ʼ����־λ
    int _badCntC = 0;
	float _deltaTime_ms = 0.0f, _nowTime_ms = 0.0f, _lastTime_ms = 0.0f;		//��λms
	cv::Point3f _lastOriginData;
    cv::Point3f _maxLimit;
    cv::Point3f _jumpLimit;
	Eigen::VectorXd _x;		//״̬������� λ�á��ٶ�
	Eigen::MatrixXd _f;		//״̬ת�ƾ���
	Eigen::MatrixXd _p;		//״̬Э�������
	Eigen::MatrixXd _q;		//������������
	Eigen::MatrixXd _h;		//��������
	Eigen::MatrixXd _r;		//������������
};

//��������
class CurveData {
private:
	static CurveData& instance() {
		static CurveData curveData;
		return curveData;
	}

    const bool CURVE_SWITCH = true;				//���߻��ƿ���
	std::vector<float> _nowTime_ms;					//�洢������Ӧʱ��
	std::vector<cv::Point3f> _velocityOrigin;		//�洢������ٶ�
	std::vector<cv::Point3f> _velocityFilter;		//�洢������ٶ�
	std::vector<cv::Point3f> accOrigin;			//�洢���ٶ�����
	std::vector<cv::Point3f> _coordinateOrigin;		//�洢ԭʼ���������
	std::vector<cv::Point3f> _coordinatePredict;	//�洢Ԥ�����������
	std::vector<cv::Point3f> _coordinateFilter;		//�洢�˲����������
	cv::Point3f _coordinateNoBias;		//�޲�����ʱ��Ԥ�����������
	std::vector<cv::Point2f> _angleNoBias;
    std::vector<cv::Point2f> _angleRef;
    std::vector<cv::Point2f> _angleFbd;
    std::vector<float>  _angleTime;
	int _writeCnt = 0;
	const int MAX_WRITE_CNT = 300;
    bool _isEmpty = true;

public:

	/**
	 * ���������
	 * @param point	ˮƽ����㣨��Ҫ��xyz��˳���Ի�
	 */
	static void saveTime(float);
	static void saveVelocity(cv::Point3f origin, cv::Point3f Filter);
	static void saveAcc(cv::Point3f origin);
	static void saveCoordinate( cv::Point3f origin, cv::Point3f filter, cv::Point3f prediction);
	static void saveNoBiasCoordinate(cv::Point3f coordinate);
	static cv::Point3f getNoBiasCoordinate();
    static void saveAngle(cv::Point2f angleNoBias, cv::Point2f angleRef, cv::Point2f angleFbd);

	/**
	 * �������
	 */
	static void clear();

	/**
	* д�������ļ�
	* ÿһ��Ϊһ�����ݣ���ͬ���ϵ������ÿո����
	*/
	static void write();

	/**
	* �����Ƿ�Ϊ��
	*/
	static bool isEmpty();

	/**
	* �Ƿ��������߻���
	*/
	static bool isEnable();

};

class Prediction {
public:
	Prediction() {
		Eigen::MatrixXd VinputP(6, 6);		//��ʼ��״̬Э�������
		VinputP <<	1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
					0.0, 0.0, 0.0, 1.0, 0.0, 1.0;

		Eigen::MatrixXd VinputQ(6, 6);		//��ʼ��������������
        VinputQ <<	7.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 7.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 200.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 5.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 200.0;

		Eigen::MatrixXd VinputH(3, 6);		//��ʼ���۲����
		VinputH <<	1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		Eigen::MatrixXd VinputR(3, 3);		//��ʼ���۲���������
        VinputR <<	1000.0, 0.0, 0.0,
                    0.0, 1000.0, 0.0,
                    0.0, 0.0, 1000.0;

		_velocityFilter.InitParam(VinputP, VinputQ, VinputH, VinputR);
        _velocityFilter.setLimit(cv::Point3f(MAX_VELOCITY_XZ, MAX_VELOCITY_Y, MAX_VELOCITY_XZ), cv::Point3f(FILTER_V_MAX, FILTER_V_MAX, FILTER_V_MAX));

        Eigen::MatrixXd PinputP(6, 6);		//��ʼ��״̬Э�������
		PinputP <<	1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
					0.0, 0.0, 0.0, 1.0, 0.0, 1.0;

        Eigen::MatrixXd PinputQ(6, 6);		//��ʼ��������������
		PinputQ <<	10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 10.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 50.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 50.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 50.0;

		Eigen::MatrixXd PinputH(3, 6);		//��ʼ���۲����
		PinputH <<	1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		Eigen::MatrixXd PinputR(3, 3);		//��ʼ���۲���������
		PinputR <<	1000.0, 0.0, 0.0,
					0.0, 1000.0, 0.0,
					0.0, 0.0, 1000.0;

		_predictionFilter.InitParam(PinputP, PinputQ, PinputH, PinputR);
        _predictionFilter.setLimit(cv::Point3f(MAX_COORDINATE_XZ, MAX_COORDINATE_Y, MAX_COORDINATE_XZ), cv::Point3f(FILTER_COORDINATE_MAX, FILTER_COORDINATE_MAX, FILTER_COORDINATE_MAX));
    }

	~Prediction() {};

	/**
	 * pitch��Ƕȼ���
	 * @param coordinate	��������ϵ�µ����� ��λΪmm
	 * @param shootSpeed	�������
	 */
	float angleCalculation(cv::Point3f coordinate, float shootSpeed);
	/**
	 * �������������Ƶ�pitch��Ƕȼ���
	 * @param coordinate	��������ϵ�µ����� ��λΪmm
	 * @param shootSpeed	�������
	 */
	float airResistCalculation(cv::Point3f coordinate, float shootSpeed);
	/**
	 * Ԥ�к���
	 * @param coordinate	�������� ��λmm
	 * @param shootSpeed	�������
	 * @param coordinateV	�������ϵ��ٶ�
	 * ֻԤ��XOZƽ���ϵ�����
	 */
	cv::Point3f prediction(cv::Point3f coordinate, float time, float shootSpeed, CarType carType);
	/**
	 * ��ȡʱ����
	 * @param time ʱ�� ��λms
	 */
	void getDeltaTime(float time);
	/**
	 * ����ٶ�����ٶ�
	 * @param coordinate �������� ��λmm
	 */
	void dataCal(cv::Point3f coordinate);
	/**
	 * ������
	 * @param data �������������
	 * @param result ����Ľ��
	 */
	void subMethod(float data[][3], cv::Point3f& result);

	cv::Point3f Iteration(cv::Point3f coordinate_m, float shootSpeed, CarType carType);

private:

    float _timeBias = 0.1f;												//���ǵ���ʵ�еĸ����ӳ������벹����
	float _deltaTime_ms = 0, _nowTime_ms = 0.0f, _lastTime_ms = 0.0f;

	KalmanFilter _velocityFilter;
	KalmanFilter _predictionFilter;

	bool _initFlag = false;											//��ʼ����־λ
	int _dataCnt = 0;												//���ݼ���
    float _data_m[V_CAL_MAX_VAL][3] = { 0.0f };					//�������ݣ���λmm
	float _velo_s[V_CAL_MAX_VAL][3] = { 0.0f };						//�ٶ����ݣ���λm/s
	cv::Point3f _V = cv::Point3f(0.0f, 0.0f, 0.0f);					//x y z�������ٶ� ��yĿǰ��ʹ�ã���Ϊ0��
	cv::Point3f _lastV = cv::Point3f(0.0f, 0.0f, 0.0f);
	cv::Point3f _A = cv::Point3f(0.0f, 0.0f, 0.0f);					//x y z��������ٶȣ�yĿǰ��ʹ�ã���Ϊ0��
	cv::Point3f _lastA = cv::Point3f(0.0f, 0.0f, 0.0f);
	float _deltaTime_s[V_CAL_MAX_VAL - 1] = { 0.0f };				//ʱ��������λs
};

#pragma once
