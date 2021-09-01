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
#define TINY            1.0e-10f     //Ϊ��ֹ0����ĸ�����뼫С��

class AngleFactory {
public:

    AngleFactory(const string &serialNumber);

    ~AngleFactory() {}

    /**
     * �õ�������
     * @param targetRect Ŀ�����
     * @param armorType װ�װ�����
     * @param distinguishMode ʶ��ģʽ
     * @param carType ��������
     * @param pitchAngle pitch��Ƕ�
     * @param yawAngle yaw��Ƕ�
     */

    void calculateFinalResult(const cv::RotatedRect& targetRect, ArmorType armorType, DistinguishMode distinguishMode, CarType carType, float pitchAngle, float yawAngle, float shootSpeed, bool sameTargetFlag, int targetWidth, bool topFlag = false);

    /**
     * װ�������
     * @param TX2Cmd �������ݽṹ��
     */
    void getAngleCalculateData(TX2CmdStruct *TX2Cmd, Stm32CmdStruct Stm32Cmd);

    /**
     * ���÷�������
     * @param TX2Cmd �������ݽṹ��
     */
    static void resetAngleCalculateData(TX2CmdStruct *TX2Cmd);

    /**
     * �������ʶ�����
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

    //����ṹ��
    ArmorCoordinateSolver::CoordinateStruct _coordinateStruct;
    BuffCoordinateSolver::CoordinateStruct _buffCoordinateStruct;

    /**
     * ���Yaw��Ƕȣ����޴���
     * �����Ǵ����ĽǶ����ݲ����360��-360ͻ��Ϊ0��ֻ�᲻�����ӻ���٣�
     * �����Ҫ���ݹ�������������ǽǶ�����������������Ƕ�
     * @param float x,y  XOZƽ���ϵ�����
     * ��Ϊ���Ϸ�����(0,0)������INF
     */
    float getYawAngle(float x, float z, float angle);

    /**
     * ����η��̷�
     * @param shootSpeed          ����
     * @param pitchAngle        pitch��Ƕ�
     * @param coordinateActual  ��������ϵ�µ����� ��λmm  
     */
    float equationCalculation(float shootSpeed, float pitchAngle, cv::Point3f coordinateActual);


    /**
     * �����������Ľ��ƽ���η��̷�
     * @param shootSpeed          ����
     * @param pitchAngle        pitch��Ƕ�
     * @param coordinateActual  ��������ϵ�µ����� ��λmm
     */
    float airResistCalculation(float shootSpeed, float pitchAngle, cv::Point3f coordinateActual);

    /**
    * ţ�ٵ�����
    * @param Stm32Cmd  32�˴���������
    */
    float newtonIterationMethod(Stm32CmdStruct stm32Cmd);

    /**
     * �����ߺ�������
     * @param resRadian �������µ�pitch�������Ƕ�
     * @param functionValue ����ֵ����ֵԽС�����赽��װ�װ崦�ڴ�ֱ�����ϵĲ�ֵԽС
     * @param derivativeValue �Ժ����󵼵õ��Ĺ�ʽ
     * @param p ˮƽ����
     * @param h ��ֱ����
     * @param v ������� 
     */
    void ballisticParabolaFunCalculation(float resRadian, float* funtionValue, float* derivativeValue, float p, float h, float v);
};

#endif

