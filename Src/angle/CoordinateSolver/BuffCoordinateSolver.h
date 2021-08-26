#ifndef _BUFF_COORDINATE_SOLVER_H_
#define _BUFF_COORDINATE_SOLVER_H_

#include "ArmorCoordinateSolver.h"


#define FRONT_BACK_PARA    2.0f
#define DELAY_TIME    0.4f
#define FRONT_BACK_SIN    0.05f

class BuffCoordinateSolver : public ArmorCoordinateSolver {
private:
    /**
     * ����������ݽṹ��
     */
    struct BuffSolverStruct {
        bool sameTargetFlag;//���ҶƬ�л���־λ
        bool captureR_flag;//Բ��Rʶ���־λ
        bool shootSpeedLevel = false;//���ٵȼ���־λ
        cv::Point2f R_2D_Point;//Բ��R��������
        cv::RotatedRect targetRect;//Ŀ��װ�װ����
    };
public:
    /**
    * ���캯��
    */
    BuffCoordinateSolver(ArmorRealData armorRealData, CameraCalibrationStruct &calibrationData);
    /**
    * ��������
    */
    ~BuffCoordinateSolver() {}

    /**
     * �ǶȽ����ⲿ����
     * @param verticesToDraw Ŀ����ε��ĸ���
     * @param pitchAngle pitch�Ƕ�
     * @param yawAngle yaw�Ƕ�
     * @param carType ��������
     */
    CoordinateStruct calculateWorldCoordinate(cv::Point2f *verticesToDraw, float pitchAngle, float yawAngle, CarType carType);


    /**
    * ͨ��ʶ��ģʽ�жϴ�С����Լ��������ñ�־λ
    * @param distinguishMode ʶ��ģʽ
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
     * �������ʶ�����
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
        if((int)shootSpeed>22){//���øߵ�����ģʽ
            _buffSolverStruct.shootSpeedLevel = 1;
        }
        else{
            _buffSolverStruct.shootSpeedLevel = 0;
           // cout<<_buffSolverStruct.shootSpeedLevel<<"level"<<endl;
        }
    }



    /**
    * ���⴫��Ԥ�д����
    */
    cv::Point2f getPronosisCoordinate() {
        return _predictCoordinate;
    }


    /**
    * ���⴫���Ƿ��л�ҶƬ��־λ
    */
    bool getSameTargetFlag() {
        return _sameTargetFlag;
    }

    /**
    * ���⴫��ʶ��������־λ
    */
    bool getCaptureFlag() const {
        return _captureFlag;
    }
private:
    /**
     * �����ת����
     */
    enum DIR_OF_ROTA {
        STOP = 0,
        CLOCKWISE,
        ANTICLOCKWISE,
        UNKNOW,
    };
    /**
     * ����ٶ��ص� ����/����
     */
    enum SPEED_TYPE {
        SPEED_UP = 0,
        SPEED_DOWN
    };
    /**
     * ���ת�ٽṹ��
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
    bool _isBig = false;//�����־λ
    bool _sameTargetFlag = false;//�Ƿ��л�ҶƬ��־λ
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
    bool _resetBuff = false;//����Ԥ�б�־λ
    bool _buffInit = false;//�Ƿ��ʼ���ɹ���־λ
    vector<float> _buffAngleList;
    DIR_OF_ROTA _buffDirOfRota = UNKNOW;
    cv::Point2f _predictCoordinate;
    cv::Point2f _predictBias = cv::Point2f(0.0f, 0.0f);
    BuffBias _buffBias = NO_CHANGE;

    /**
     * ������Ԥ�е�
     */
    void makeFinalVertices(cv::Point2f *vertices);

    /**
     * �����ת�����ж�
     */
    void buffCoordinateCalculateInit();

    /**
     * ������ת��������Ԥ�е�
     */
    void buffPredictCoordinate2D(cv::Point2f *vertices);

    /**
     * ����Ԥ�ƽǶ�
     */
    float calculateAddAngle();

    /**
     * ����ת��
     */
    void calculateRotateSpeed();

    /**
     * �������ʱ��
     */
    float calculateShootTime();

};


#endif
