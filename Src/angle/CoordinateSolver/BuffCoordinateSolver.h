#ifndef _BUFF_COORDINATE_SOLVER_H_
#define _BUFF_COORDINATE_SOLVER_H_

#include "ArmorCoordinateSolver.h"


#define FRONT_BACK_PARA    2.0f
#define DELAY_TIME    0.4f
#define FRONT_BACK_SIN    0.05f

class BuffCoordinateSolver : public ArmorCoordinateSolver {
private:
    /**
     * 神符决策数据结构体
     */
    struct BuffSolverStruct {
        bool sameTargetFlag;//神符叶片切换标志位
        bool captureR_flag;//圆心R识别标志位
        bool shootSpeedLevel = false;//射速等级标志位
        cv::Point2f R_2D_Point;//圆心R中心坐标
        cv::RotatedRect targetRect;//目标装甲板矩形
    };
public:
    /**
    * 构造函数
    */
    BuffCoordinateSolver(ArmorRealData armorRealData, CameraCalibrationStruct &calibrationData);
    /**
    * 析构函数
    */
    ~BuffCoordinateSolver() {}

    /**
     * 角度解算外部调用
     * @param verticesToDraw 目标矩形的四个点
     * @param pitchAngle pitch角度
     * @param yawAngle yaw角度
     * @param carType 车辆类型
     */
    CoordinateStruct calculateWorldCoordinate(cv::Point2f *verticesToDraw, float pitchAngle, float yawAngle, CarType carType);


    /**
    * 通过识别模式判断大小神符以及设置重置标志位
    * @param distinguishMode 识别模式
    */
    void setLastDistinguishMode(DistinguishMode distinguishMode) {
        ArmorCoordinateSolver::setLastDistinguishMode(distinguishMode);
        _isBig = distinguishMode == TX2_DISTINGUISH_BIG_BUFF;
        //_resetBuff = ArmorCoordinateSolver::getResetActualCoordinate();
        static int lastDistinguishMode = 0;
        //cout<<"mode"<<lastDistinguishMode<<" "<<distinguishMode<<endl;
        if(lastDistinguishMode!=distinguishMode){
            _resetBuff = true;
           // cout<<"reset"<<endl;
        }
        lastDistinguishMode = distinguishMode;
    }

    /**
     * 传递神符识别参数
     * @param sameTargetFlag
     * @param captureR_flag
     * @param R_2D_Point
     * @param targetRect
     */
    void setBuffSolverStruct(bool sameTargetFlag, bool captureR_flag, const cv::Point2f &R_2D_Point, cv::RotatedRect targetRect, BuffBias buffBias,int shootSpeed) {
        _buffSolverStruct.sameTargetFlag = sameTargetFlag;
        _buffSolverStruct.captureR_flag = captureR_flag;
        _buffSolverStruct.R_2D_Point = R_2D_Point;
        _buffSolverStruct.targetRect = targetRect;
        _buffBias = buffBias;
        //cout<<"speed"<<(int)shootSpeed<<endl;
        if((int)shootSpeed>22){//设置高低射速模式
            _buffSolverStruct.shootSpeedLevel = 1;
        }
        else{
            _buffSolverStruct.shootSpeedLevel = 0;
           // cout<<_buffSolverStruct.shootSpeedLevel<<"level"<<endl;
        }
    }



    /**
    * 向外传输预判打击点
    */
    cv::Point2f getPronosisCoordinate() {
        return _predictCoordinate;
    }


    /**
    * 向外传输是否切换叶片标志位
    */
    bool getSameTargetFlag() {
        return _sameTargetFlag;
    }

    /**
    * 向外传输识别锁定标志位
    */
    bool getCaptureFlag() const {
        return _captureFlag;
    }
private:
    /**
     * 神符旋转方向
     */
    enum DIR_OF_ROTA {
        STOP = 0,
        CLOCKWISE,
        ANTICLOCKWISE,
        UNKNOW,
    };
    /**
     * 大符速度特点 加速/减速
     */
    enum SPEED_TYPE {
        SPEED_UP = 0,
        SPEED_DOWN
    };
    /**
     * 大符转速结构体
     */
    struct RotateSpeed {
        float lastRotateSpeed = 0.0f;
        float nowRotateSpeed = 0.0f;
        float realRotateSpeed = 0.0f;
        SPEED_TYPE speedType;
    };

    struct SpeedRange {
        float nowMinSpeed = 100.0f;
        float realMinSpeed = 0.0f;
        float nowMaxSpeed = 0.0f;
        float realMaxSpeed = 0.0f;
        int minSameNumber = 0;
        int maxSameNumber = 0;
        bool minSpeedFlag = false;
        bool maxSpeedFlag = false;
    };
    struct SineFunction {
        float amplitude = 0.785f;
        float rotateIndex = 1.884f;
        float para = 1.305f;
    };

    bool _captureFlag = false;
    bool _isBig = false;//大符标志位
    bool _sameTargetFlag = false;//是否切换叶片标志位
    BuffSolverStruct _buffSolverStruct;
    ArmorRealData _buffArmor = ArmorDataFactory::getBuffArmor();
    RotateSpeed _rotateSpeed;
    SpeedRange _speedRange;
    SineFunction _sineFunction;
    ParaCircle _paraCircle = BuffParaFactory::getParaCircle();
    float _circleAngle180 = 0.0f;
    float _circleAngle360 = 0.0f;
    float _realAddAngle = 0.0f;
    float _para = 0.0f;
    float _armorRatio = 1.78f;
    float _delayTime = 0.4f;
    bool _resetBuff = false;//重置预判标志位
    bool _buffInit = false;//是否初始化成功标志位
    vector<float> _buffAngleList;
    DIR_OF_ROTA _buffDirOfRota = UNKNOW;
    cv::Point2f _predictCoordinate;
    cv::Point2f _predictBias = cv::Point2f(0.0f, 0.0f);
    BuffBias _buffBias = NO_CHANGE;

    /**
     * 获得神符预判点
     */
    void makeFinalVertices(cv::Point2f *vertices);

    /**
     * 神符旋转方向判断
     */
    void buffCoordinateCalculateInit();

    /**
     * 根据旋转方向解算出预判点
     */
    void buffPredictCoordinate2D(cv::Point2f *vertices);

    /**
     * 计算预制角度
     */
    float calculateAddAngle();

    /**
     * 计算转速
     */
    void calculateRotateSpeed();

    /**
     * 计算射击时间
     */
    float calculateShootTime();

};


#endif
