#ifndef _ARMOR_COORDINATE_SOLVER_H_
#define _ARMOR_COORDINATE_SOLVER_H_

#include <angle/PNP/PNPSolver.h>
#include <angle/KalmanPredict/KalmanPredict.h>
#include "tool/Conf.h"

class ArmorCoordinateSolver {

public:

    /**
    * 坐标结构体
    */
    struct CoordinateStruct {
        cv::Point3f worldOriginInPZT = cv::Point3f(0.0f, 0.0f, 0.0f);   //相机坐标系下的坐标
        cv::Point3f actualCoordinate = cv::Point3f(0.0f, 0.0f, 0.0f);   //固有坐标系下的坐标
        float coordinateTime;
    };

    ArmorCoordinateSolver(ArmorRealData armorRealData, CameraCalibrationStruct& calibrationData);

    ~ArmorCoordinateSolver() {}

    /**
     * 设置上一帧的识别模式
     * @param distinguishMode 识别模式
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
     * 角度解算外部调用
     * @param vertices 目标矩形的四个点
     * @param pitchAngle pitch角度
     * @param yawAngle yaw角度
     * @param carType 车辆类型
     */
    CoordinateStruct calculateWorldCoordinate(cv::Point2f* vertices, float pitchAngle, float yawAngle, float shootSpeed, CarType carType, int targetWidth, bool topFlag = false);

    /**
     * 根据矩形四个点计算真实坐标
     * @param _vertices 装甲板四个点
     * @param carType 车辆类型
     */
    void realCoordinateCalculate(cv::Point2f _vertices[4], CarType carType);

    /**
     * 旋转真实坐标
     */
    void actualCoordinateSolver();

    /**
     * 装填发送坐标
     * @return 最终结果结构体
     */
    CoordinateStruct mixCoordinate(float shootSpeed);

    /**
   * 装填发送坐标(为神符重载)
   * @return 最终结果结构体
   */
    CoordinateStruct mixCoordinate();

    /**
     * 获取是否重置pitch，yaw旋转角度
     * @return
     */
    bool getResetActualCoordinate() const {
        return _resetActualCoordinate;
    }

    float _pitchAngle;
    float _yawAngle;

protected:
    cv::Point3f _worldOriginInPZTActual; //固有坐标系下的坐标
    cv::Point3f _lastWorldOriginInPZTActual;
    cv::Point3f _worldOriginInPZT;      //世界坐标在平行相机坐标系位置(云台中心为原点)
    cv::Point3f _lastWorldOriginInPZT;
    cv::Point3f _worldOriginInBarrel;   //世界坐标在平行相机坐标系位置(枪管出口为原点)
    cv::Point3f _worldOriginInCamera;   //世界坐标在相机坐标系位置(相机为原点)

    bool _topFlag;
    bool _lastTopFlag = false;

    //top
    vector<float> _xHistory;
    vector<float> _yHistory;
    vector<float> _zHistory;

    float _maxCoordinate = 0.0;
    float _minCoordinate = 0.0;

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

