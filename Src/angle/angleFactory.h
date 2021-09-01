#ifndef _ANGLE_FACTORY_H_
#define _ANGLE_FACTORY_H_

#include <serial/serial.h>
#include <angle/CoordinateSolver/BuffCoordinateSolver.h>

#define MM_TO_M_TRANS(p)               (float)(p) / 1000.0f
#define M_TO_MM_TRANS(p)	            (float)(p) * 1000.0f

#define ANGLE_TO_RADIAN(p)                  (p) * (float)CV_PI / 180.0f
#define RADIAN_TO_ANGLE(p)                  (p) * 180.0f / (float)CV_PI

#define GRAVITY			9.80565f	// m/s^2
#define PI              3.14159265358979f
#define TINY            1.0e-10f     //为防止0做分母，引入极小项

class AngleFactory {
public:

    AngleFactory(const string &serialNumber);

    ~AngleFactory() {}

    /**
     * 得到解算结果
     * @param targetRect 目标矩形
     * @param armorType 装甲板类型
     * @param distinguishMode 识别模式
     * @param carType 车辆类型
     * @param pitchAngle pitch轴角度
     * @param yawAngle yaw轴角度
     */

    void calculateFinalResult(const cv::RotatedRect& targetRect, ArmorType armorType, DistinguishMode distinguishMode, CarType carType, float pitchAngle, float yawAngle, float shootSpeed, bool sameTargetFlag, int targetWidth, bool topFlag = false);

    /**
     * 装填发送数据
     * @param TX2Cmd 发送数据结构体
     */
    void getAngleCalculateData(TX2CmdStruct *TX2Cmd, Stm32CmdStruct Stm32Cmd);

    /**
     * 重置发送数据
     * @param TX2Cmd 发送数据结构体
     */
    static void resetAngleCalculateData(TX2CmdStruct *TX2Cmd);

    /**
     * 设置神符识别参数
     * @param sameTargetFlag
     * @param captureR_flag
     * @param R_2D_Point
     * @param targetRect
     */
    void setBuffSolverStruct(bool sameTargetFlag, bool captureR_flag, const cv::Point2f& R_2D_Point, cv::RotatedRect targetRect, BuffBias buffBias,int shootSpeed) {
        _buffCoordinateSolver->setBuffSolverStruct(sameTargetFlag, captureR_flag, R_2D_Point, targetRect, buffBias,shootSpeed);
    }

    bool getCaptureFlag() {
        return _buffCoordinateSolver->getCaptureFlag();
    }

    cv::Point2f *getBuffVertices() {
        return _buffVertices;
    }

    cv::Point2f getPronosisCoordinate() {
        return _buffCoordinateSolver->getPronosisCoordinate();
    }

    float get_yawAngle() {
        return _yawAngle;
    }

    static bool getBuffTargetNoChangeFlag();


private:
//    float _advanceValue;
    float _yawAngle = 0.0;
    static bool _buffTargetNoChangeFlag;
    cv::Point2f _buffVertices[4];
    CarType _carType;
    ArmorCoordinateSolver *_smallArmorCoordinateSolver;
    ArmorCoordinateSolver *_bigArmorCoordinateSolver;
    BuffCoordinateSolver *_buffCoordinateSolver;

    //坐标结构体
    ArmorCoordinateSolver::CoordinateStruct _coordinateStruct;
    BuffCoordinateSolver::CoordinateStruct _buffCoordinateStruct;

    /**
     * 获得Yaw轴角度（象限处理）
     * 陀螺仪传来的角度数据不会从360或-360突变为0，只会不断增加或减少，
     * 因此需要根据固有坐标和陀螺仪角度来算出陀螺仪期望角度
     * @param float x,y  XOZ平面上的坐标
     * 若为不合法坐标(0,0)，返回INF
     */
    float getYawAngle(float x, float z, float angle);

    /**
     * 解二次方程法
     * @param shootSpeed          弹速
     * @param pitchAngle        pitch轴角度
     * @param coordinateActual  固有坐标系下的坐标 单位mm  
     */
    float equationCalculation(float shootSpeed, float pitchAngle, cv::Point3f coordinateActual);


    /**
     * 带空气阻力的近似解二次方程法
     * @param shootSpeed          弹速
     * @param pitchAngle        pitch轴角度
     * @param coordinateActual  固有坐标系下的坐标 单位mm
     */
    float airResistCalculation(float shootSpeed, float pitchAngle, cv::Point3f coordinateActual);

    /**
    * 牛顿迭代法
    * @param Stm32Cmd  32端传来的数据
    */
    float newtonIterationMethod(Stm32CmdStruct stm32Cmd);

    /**
     * 抛物线函数运算
     * @param resRadian 弧度制下的pitch轴期望角度
     * @param functionValue 函数值，该值越小，弹丸到达装甲板处在垂直距离上的差值越小
     * @param derivativeValue 对函数求导得到的公式
     * @param p 水平距离
     * @param h 垂直距离
     * @param v 弹丸初速 
     */
    void ballisticParabolaFunCalculation(float resRadian, float* funtionValue, float* derivativeValue, float p, float h, float v);
};

#endif

