#ifndef _ARMOR_COORDINATE_SOLVER_H_
#define _ARMOR_COORDINATE_SOLVER_H_

#include <angle/PNP/PNPSolver.h>
#include <angle/KalmanPredict/KalmanPredict.h>
#include "tool/Conf.h"

class ArmorCoordinateSolver {

public:

    /**
    * ����ṹ��
    */
    struct CoordinateStruct {
        cv::Point3f worldOriginInPZT = cv::Point3f(0.0f, 0.0f, 0.0f);   //�������ϵ�µ�����
        cv::Point3f actualCoordinate = cv::Point3f(0.0f, 0.0f, 0.0f);   //��������ϵ�µ�����
        float coordinateTime;
    };

    ArmorCoordinateSolver(ArmorRealData armorRealData, CameraCalibrationStruct& calibrationData);

    ~ArmorCoordinateSolver() {}

    /**
     * ������һ֡��ʶ��ģʽ
     * @param distinguishMode ʶ��ģʽ
     */
    void setLastDistinguishMode(DistinguishMode distinguishMode) {
        _distinguishMode = distinguishMode;
        if (_lastDistinguishMode != _distinguishMode) {
            _resetActualCoordinate = true;
        }
        _lastDistinguishMode = _distinguishMode;
        //cout<<"_resetActualCoordinate = "<<_resetActualCoordinate<<endl;
    }

    /**
     * �ǶȽ����ⲿ����
     * @param vertices Ŀ����ε��ĸ���
     * @param pitchAngle pitch�Ƕ�
     * @param yawAngle yaw�Ƕ�
     * @param carType ��������
     */
    CoordinateStruct calculateWorldCoordinate(cv::Point2f* vertices, float pitchAngle, float yawAngle, float shootSpeed, CarType carType, int targetWidth);

    /**
     * ���ݾ����ĸ��������ʵ����
     * @param _vertices װ�װ��ĸ���
     * @param carType ��������
     */
    void realCoordinateCalculate(cv::Point2f _vertices[4], CarType carType);

    /**
     * ��ת��ʵ����
     */
    void actualCoordinateSolver();

    /**
     * װ�������
     * @return ���ս���ṹ��
     */
    CoordinateStruct mixCoordinate(float shootSpeed);

    /**
   * װ�������(Ϊ�������)
   * @return ���ս���ṹ��
   */
    CoordinateStruct mixCoordinate();

    /**
     * ��ȡ�Ƿ�����pitch��yaw��ת�Ƕ�
     * @return
     */
    bool getResetActualCoordinate() const {
        return _resetActualCoordinate;
    }

    float _pitchAngle;
    float _yawAngle;

protected:
    cv::Point3f _worldOriginInPZTActual; //��������ϵ�µ�����
    cv::Point3f _lastWorldOriginInPZTActual;
    cv::Point3f _worldOriginInPZT;      //����������ƽ���������ϵλ��(��̨����Ϊԭ��)
    cv::Point3f _lastWorldOriginInPZT;
    cv::Point3f _worldOriginInBarrel;   //����������ƽ���������ϵλ��(ǹ�ܳ���Ϊԭ��)
    cv::Point3f _worldOriginInCamera;   //�����������������ϵλ��(���Ϊԭ��)

    CarType _carType;
    CoordinateStruct _coordinateStruct;
    PNPSolver *_PNPSolver;
    KalmanFilter _coordinateFilter;
    Prediction _prediction;
    bool _resetActualCoordinate = false;
    bool _resetBuffFilterFlag = false;
    int _targetWidth = 0;

    DistinguishMode _distinguishMode = TX2_STOP;
    DistinguishMode _lastDistinguishMode = TX2_STOP;
};


#endif

