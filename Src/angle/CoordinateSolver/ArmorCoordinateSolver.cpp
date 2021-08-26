#include "ArmorCoordinateSolver.h"
#include <angle/KalmanPredict/KalmanPredict.h>
#include "angle/angleFactory.h"

ArmorCoordinateSolver::ArmorCoordinateSolver(ArmorRealData armorRealData, CameraCalibrationStruct& calibrationData) {
    _PNPSolver = new PNPSolver(calibrationData);

    //װ�װ�ʵ�ʳߴ���װ����λ����
    float halfWidth = armorRealData.width / 2;
    float halfLength = armorRealData.length / 2;
    _PNPSolver->Points3D.push_back(cv::Point3f(-halfLength, -halfWidth, 0));       //P1 ���ϵ�
    _PNPSolver->Points3D.push_back(cv::Point3f(halfLength, -halfWidth, 0));        //P2 ���ϵ�
    _PNPSolver->Points3D.push_back(cv::Point3f(halfLength, halfWidth, 0));         //P3 ���µ�
    _PNPSolver->Points3D.push_back(cv::Point3f(-halfLength, halfWidth, 0));        //P4 ���µ�

    _pitchAngle = 0.0f;
    _yawAngle = 0.0f;

    Eigen::MatrixXd inputP(6, 6);		//��ʼ��״̬Э�������
    inputP <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 1.0;

    Eigen::MatrixXd inputQ(6, 6);		//��ʼ��������������
    inputQ <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 50.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 50.0;

    Eigen::MatrixXd inputH(3, 6);		//��ʼ���۲����
    inputH <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    Eigen::MatrixXd inputR(3, 3);		//��ʼ���۲���������
    inputR <<   100.0, 0.0, 0.0,
                0.0, 100.0, 0.0,
                0.0, 0.0, 100.0;

    _coordinateFilter.InitParam(inputP, inputQ, inputH, inputR);
    _coordinateFilter.setLimit(cv::Point3f(MAX_COORDINATE_XZ, MAX_COORDINATE_Y, MAX_COORDINATE_XZ), cv::Point3f(FILTER_COORDINATE_MAX, FILTER_COORDINATE_MAX, FILTER_COORDINATE_MAX));
}

ArmorCoordinateSolver::CoordinateStruct ArmorCoordinateSolver::calculateWorldCoordinate(cv::Point2f* vertices, float pitchAngle, float yawAngle, float shootSpeed, CarType carType,int targetWidth) {
    _pitchAngle = pitchAngle;
    _yawAngle = yawAngle;
    _targetWidth = targetWidth;
    _carType = carType;

    realCoordinateCalculate(vertices, carType);

    actualCoordinateSolver();

    return mixCoordinate(shootSpeed);
}

void ArmorCoordinateSolver::realCoordinateCalculate(cv::Point2f *_vertices, CarType carType) {
    //���Ƚ�λ�˹������ڵ�������������������
    _PNPSolver->Points2D.clear();
    //��Ӿ��ε�4��������Ϊ������
    for (int i = 0; i < 4; i++) {
        _PNPSolver->Points2D.push_back(_vertices[i]);
    }

    _PNPSolver->Solve(PNPSolver::METHOD::ITERATIVE);

    cout << " Depth no fix: " << fabs(_PNPSolver->Position_OwInC.z) << endl;

    float nowTime_ms;
    static float lastTime_ms = 0;
    nowTime_ms = cv::getTickCount() / cv::getTickFrequency() * 1000;	//ms
    float deltaTime_ms = (float)(nowTime_ms - lastTime_ms);				//ms
    lastTime_ms = nowTime_ms;

    // ��ȡװ�װ�������λ��
    cout << " Width: " << _targetWidth << endl;

    static cv::Point3f lastFix = { 0.0f };
    cv::Point3f fix = { 0.0f };
    fix.x = _PNPSolver->Position_OwInC.x;
    fix.y = _PNPSolver->Position_OwInC.y;
    fix.z = fabs(_PNPSolver->Position_OwInC.z);
    if(_distinguishMode != TX2_DISTINGUISH_BUFF&&_distinguishMode != TX2_DISTINGUISH_BIG_BUFF){
        float alpha = (1 - 1.0f / (_targetWidth + 1));      //����ϵ��
        //fix.x = fix.x * alpha;
        //fix.y = fix.y * alpha;
        //fix.z = fix.z * alpha;
    }

    cout << " Depth fix:  " << fix.z << endl;

    //7m�ڲ��˲�
    if (fabs(_PNPSolver->Position_OwInC.z) <= 7000) {
        
    }
    //ʹ�õ�ͨ�˲�ƽ������
    else {
        float lowPass = 0.5;
        
        if (deltaTime_ms <= 300) {
            fix.x = lastFix.x * lowPass + fix.x * (1 - lowPass);
            fix.y = lastFix.y * lowPass + fix.y * (1 - lowPass);
            fix.z = lastFix.z * lowPass + fix.z * (1 - lowPass);
        }
    }
    lastFix.x = fix.x;
    lastFix.y = fix.y;
    lastFix.z = fix.z;

    cout << " Depth fix filter:  " << fix.z << endl;

    _worldOriginInCamera.x = fix.x;
    _worldOriginInCamera.y = fix.y;
    _worldOriginInCamera.z = fix.z;

    //���ݳ������ͻ�ȡװ�װ����ǹ�ܳ����ڵ���ʵ������Ϣ
    InstallDataFactory::getRealCoordinateDataByCarType(_worldOriginInPZT, _worldOriginInBarrel, _worldOriginInCamera, carType);
    if(isnan(_worldOriginInPZT.x) || isnan(_worldOriginInPZT.y) || isnan(_worldOriginInPZT.z)) {
        //LOG::debug("_worldOriginInPZT is nan!");
        cout << "_worldOriginInPZT is nan!" << endl;
    }
}

void ArmorCoordinateSolver::actualCoordinateSolver() {

    double thetaX = (double) _pitchAngle;
    double thetaY = (double) _yawAngle;
    double x = _worldOriginInPZT.x;
    double y = _worldOriginInPZT.y;
    double z = _worldOriginInPZT.z;

    PNPSolver::CodeRotateByZ(x, y, 0, x, y);
    PNPSolver::CodeRotateByX(y, z, thetaX, y, z);  
    PNPSolver::CodeRotateByY(x, z, thetaY, x, z);
    
    _worldOriginInPZTActual.x = (float) x;
    _worldOriginInPZTActual.y = (float) y;
    _worldOriginInPZTActual.z = (float) z;

    if(isnan(_worldOriginInPZTActual.x) || isnan(_worldOriginInPZTActual.y) || isnan(_worldOriginInPZTActual.z)) {

        //LOG::debug("_worldOriginInPZTActual is nan!");
        cout << "_worldOriginInPZTActual is nan!" << endl;
        cout << "_pitchAngle: " << _pitchAngle << endl;
        cout << "_yawAngle: " << _yawAngle << endl << endl;
    }

    //��������ϵ����
    //cout << "_worldOriginInPZTActual: " << _worldOriginInPZTActual << endl << endl;

}

ArmorCoordinateSolver::CoordinateStruct ArmorCoordinateSolver::mixCoordinate(float shootSpeed) {

    //װ���������
    _coordinateStruct.worldOriginInPZT.x = _worldOriginInPZT.x;
    _coordinateStruct.worldOriginInPZT.y = _worldOriginInPZT.y;
    _coordinateStruct.worldOriginInPZT.z = _worldOriginInBarrel.z;

    //cout << "_worldOriginInPZT : " << _worldOriginInPZT << endl << endl;

    //�Թ�������ϵ��������˲�
    cv::Point3f Kcoordinate = cv::Point3f(MM_TO_M_TRANS(_worldOriginInPZTActual.x) , MM_TO_M_TRANS(_worldOriginInPZTActual.y), MM_TO_M_TRANS(_worldOriginInPZTActual.z));
    Kcoordinate = _coordinateFilter.KPredictionRun(Kcoordinate);
    //��λת��
    Kcoordinate.x = M_TO_MM_TRANS(Kcoordinate.x);
    Kcoordinate.y = M_TO_MM_TRANS(Kcoordinate.y);
    Kcoordinate.z = M_TO_MM_TRANS(Kcoordinate.z);
    
    //�������˲����������Ϣ��x,z��
    //cout << "Kcoordinate: " << Kcoordinate << endl;

    //װ���������ṹ��
    _coordinateStruct.actualCoordinate = Kcoordinate;
    _coordinateStruct.coordinateTime = _coordinateFilter.getKFtime();
    CurveData::saveTime(_coordinateStruct.coordinateTime);              //�洢����ʱ��

    float x_m, y_m, z_m;
    x_m = MM_TO_M_TRANS(_coordinateStruct.actualCoordinate.x);
    y_m = MM_TO_M_TRANS(_coordinateStruct.actualCoordinate.y);
    z_m = MM_TO_M_TRANS(_coordinateStruct.actualCoordinate.z);

    //�Ƿ����Ԥ��
    switch (_carType) {
        case INFANTRY: 
            if (x_m * x_m + z_m * z_m <= 64.0f) {
                _coordinateStruct.actualCoordinate = _prediction.prediction(_coordinateStruct.actualCoordinate, _coordinateStruct.coordinateTime, shootSpeed, _carType);
            }
            break;
        case OLD_HERO:
        case NEW_HERO:
                _coordinateStruct.actualCoordinate = _prediction.prediction(_coordinateStruct.actualCoordinate, _coordinateStruct.coordinateTime, shootSpeed, _carType);
            break;
        case OLD_SENTRY_BELOW:
        case NEW_SENTRY_BELOW:
            _coordinateStruct.actualCoordinate = _prediction.prediction(_coordinateStruct.actualCoordinate, _coordinateStruct.coordinateTime, shootSpeed, _carType);
            break;
        case OLD_SENTRY_ABOVE:
        case NEW_SENTRY_ABOVE:
            if (x_m * x_m + z_m * z_m <= 64.0f) {
                _coordinateStruct.actualCoordinate = _prediction.prediction(_coordinateStruct.actualCoordinate, _coordinateStruct.coordinateTime, shootSpeed, _carType);
            }
            break;
        case PLANE:
            _coordinateStruct.actualCoordinate = _prediction.prediction(_coordinateStruct.actualCoordinate, _coordinateStruct.coordinateTime, shootSpeed, _carType);
            break;
        default:
            _coordinateStruct.actualCoordinate = _prediction.prediction(_coordinateStruct.actualCoordinate, _coordinateStruct.coordinateTime, shootSpeed, _carType);
            break;
    }

    CurveData::saveCoordinate(_worldOriginInPZTActual, Kcoordinate, _coordinateStruct.actualCoordinate);    //�洢��������

    return _coordinateStruct;
}

ArmorCoordinateSolver::CoordinateStruct ArmorCoordinateSolver::mixCoordinate() {
    if (AngleFactory::getBuffTargetNoChangeFlag()) {
        _resetBuffFilterFlag = false;
    }
    else {
        _resetBuffFilterFlag = true;
        _coordinateFilter.setInitFalse();   //�������˲�����
    }
    //װ���������
    _coordinateStruct.worldOriginInPZT.x = _worldOriginInPZT.x;
    _coordinateStruct.worldOriginInPZT.y = _worldOriginInPZT.y;
    _coordinateStruct.worldOriginInPZT.z = _worldOriginInBarrel.z;

    //cout << "_worldOriginInPZT: " << _worldOriginInPZTActual << endl << endl;

    //�Թ�������ϵx,z����������˲� ͬʱ����ٶ�����   
    cv::Point3f Kcoordinate = cv::Point3f(0.0f, 0.0f, 0.0f);
    //��λת��
    Kcoordinate = cv::Point3f(MM_TO_M_TRANS(_worldOriginInPZTActual.x), MM_TO_M_TRANS(_worldOriginInPZTActual.y), MM_TO_M_TRANS(_worldOriginInPZTActual.z));
    Kcoordinate = _coordinateFilter.KPredictionRun(Kcoordinate);
    //��λת��
    Kcoordinate.x = M_TO_MM_TRANS(Kcoordinate.x);
    Kcoordinate.y = M_TO_MM_TRANS(Kcoordinate.y);
    Kcoordinate.z = M_TO_MM_TRANS(Kcoordinate.z);

    //�������˲����������Ϣ��x,z��
    //cout << "KcoordinateStruct.actualCoordinate" << Kcoordinate << endl << endl;

    //װ���������ṹ��
    _coordinateStruct.actualCoordinate = Kcoordinate;
    _coordinateStruct.coordinateTime = _coordinateFilter.getKFtime();

    CurveData::saveTime(_coordinateStruct.coordinateTime);              //�洢����ʱ��

    return _coordinateStruct;
}
