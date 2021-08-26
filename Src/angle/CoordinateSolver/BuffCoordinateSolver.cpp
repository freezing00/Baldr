#include "BuffCoordinateSolver.h"

BuffCoordinateSolver::BuffCoordinateSolver(ArmorRealData armorRealData, CameraCalibrationStruct &calibrationData) : ArmorCoordinateSolver(armorRealData, calibrationData) {

}

//���Ԥ�����̺���
ArmorCoordinateSolver::CoordinateStruct BuffCoordinateSolver::calculateWorldCoordinate(cv::Point2f * verticesToDraw, float pitchAngle, float yawAngle, CarType carType) {
   _pitchAngle = pitchAngle;
    _yawAngle = yawAngle;

    //Ԥ��
    makeFinalVertices(verticesToDraw);
    //λ�˽���
    ArmorCoordinateSolver::realCoordinateCalculate(verticesToDraw, carType);
    //��ʵ�������
    ArmorCoordinateSolver::actualCoordinateSolver();
    //�˲��Լ�����װ��
    return ArmorCoordinateSolver::mixCoordinate();
}

void BuffCoordinateSolver::makeFinalVertices(cv::Point2f *vertices) {

    buffCoordinateCalculateInit();
    if (_buffDirOfRota != UNKNOW) {
        buffPredictCoordinate2D(vertices);
        _captureFlag = true;
    } else {
        _captureFlag = false;
    }
}

//Ԥ��ǰ�ĳ�ʼ��
void BuffCoordinateSolver::buffCoordinateCalculateInit() {
    //cout<<"init"<<endl;
    static float lastCircleAngle = 0.0f;
    static cv::Point2f lastTargetRectCenter = cv::Point2f(0.0f, 0.0f);
    static float circleAngleBias = 0.0f;
    _circleAngle180 = Util::getAngle(_buffSolverStruct.targetRect.center, cv::Point2f(3000.0f, _buffSolverStruct.R_2D_Point.y), _buffSolverStruct.R_2D_Point);
    //��ת�Ƕȴ���
    if (_buffSolverStruct.R_2D_Point.y < _buffSolverStruct.targetRect.center.y) {
        _circleAngle360 = 360.0f - _circleAngle180;
        _circleAngle180 = -_circleAngle180;
    } else {
        _circleAngle360 = _circleAngle180;
    }
    //�����������
    if (_resetBuff) {
        if (_buffDirOfRota != UNKNOW) {
            _buffDirOfRota = UNKNOW;
            _buffInit = false;
            _buffAngleList.resize(0);
            _resetBuff = false;
        }
    }
    circleAngleBias = _circleAngle360 - lastCircleAngle;


    //�ж��Ƿ���ͬһƬҶƬ
    if ((fabsf(circleAngleBias) > 10.0f || (fabsf(lastTargetRectCenter.y - _buffSolverStruct.targetRect.center.y) > 40.0f) || (fabsf(lastTargetRectCenter.x - _buffSolverStruct.targetRect.center.x) > 40.0f))) {
        _sameTargetFlag = false;
    }
    else {
        _sameTargetFlag = true;
    }


    if (!_buffInit) {
        //��ȥ�Ƕȴ���
        if (!_sameTargetFlag) {
            _buffAngleList.resize(0);//�Ƕȱ仯������ΪҶƬ����
        } else {
            _buffAngleList.push_back(_circleAngle360);
        }
        //�����㹻�жϷ���

        if (_buffAngleList.size() > 5) {//��������������ж�
            float averageAngle = 0.0f;
            float buff = 0.0f;
            uint8_t count = 0;
            //������
            for (size_t i = 0; i < _buffAngleList.size() / 2; i++) {
                buff = _buffAngleList[_buffAngleList.size() / 2 + i] - _buffAngleList[i];
                if (!buff) {
                    continue;
                } else {
                    averageAngle += buff;
                    count++;
                }
            }
            if (count) {
                averageAngle = averageAngle / count;
            }
            //�ж���ת����
            if (averageAngle < -1.0f) {
                _buffDirOfRota = CLOCKWISE;
            } else if (averageAngle > 1.0f) {
                _buffDirOfRota = ANTICLOCKWISE;
            } else {
                _buffDirOfRota = STOP;
            }
            _buffInit = true;
        }
    }
   // cout<<"yeah"<<_predictCoordinate<<endl;
    lastCircleAngle = _circleAngle360;
    lastTargetRectCenter = _buffSolverStruct.targetRect.center;
}

//��άԤ��
void BuffCoordinateSolver::buffPredictCoordinate2D(cv::Point2f *vertices) {
    float r = 0.0f;
    float addAngle = 0.0f;
    float resAngle = 0.0f;
    static cv::Point2f predictBias = cv::Point2f(0.0f, 0.0f);
    switch (_buffDirOfRota) {
        case STOP: {
            addAngle = 0.0f;
        }
            break;
        case CLOCKWISE: {
            addAngle = calculateAddAngle();
        }
            break;
        case ANTICLOCKWISE: {
            addAngle = -calculateAddAngle();
        }
            break;
        case UNKNOW:
            break;
    }

    if (_buffDirOfRota) {
        resAngle = -_circleAngle180 + addAngle;
        if (_circleAngle180 >= 180.0f) {
            resAngle = resAngle - 360.0f;
        } else if (resAngle < -180.0f) {
            resAngle = 360.0f + resAngle;
        }
        cv::Point2f points[4];
        _buffSolverStruct.targetRect.points(points);
        int index = 3;
        if(!_buffSolverStruct.shootSpeedLevel){
            index = 5;
        }

        //�������ⲿ�������νӿ�
        if (_buffBias == UP) {
            _buffArmor.length += index * _armorRatio;
            _buffArmor.width += index * _armorRatio;
//            cout << "_buffArmor.length" << _buffArmor.length << "," << "_buffArmor.width" << _buffArmor.width << endl;
        } else if (_buffBias == DOWN) {
            _buffArmor.length -= index * _armorRatio;
            _buffArmor.width -= index * _armorRatio;
//            cout << "_buffArmor.length" << _buffArmor.length << "," << "_buffArmor.width" << _buffArmor.width << endl;
        }

        //λ��Ԥ��
        r = Util::pointDistance(_buffSolverStruct.R_2D_Point, _buffSolverStruct.targetRect.center)*1.08;
        _predictCoordinate.x = _buffSolverStruct.R_2D_Point.x + r * cosf(resAngle * Util::PI_F() / 180.0f);
        _predictCoordinate.y = _buffSolverStruct.R_2D_Point.y + r * sinf(resAngle * Util::PI_F() / 180.0f);
    } else {
        _predictCoordinate = _buffSolverStruct.targetRect.center;
    }
    //cout<<"prediction"<<_buffSolverStruct.targetRect.center<<endl;
    predictBias= _predictCoordinate -_buffSolverStruct.targetRect.center;

    //Ϊ��ͼ���ϳ���Ԥ�е�洢����
    for (int i = 0; i < 4; i++) {
        vertices[i] = vertices[i] + predictBias;
    }
}

float BuffCoordinateSolver::calculateAddAngle() {
    float time = 0.0f;
    //����ʵʱ���ٶ�
    calculateRotateSpeed();
    //������ֿ�ʼʱ�̣���ʱ����Ϊ���ٶȺ����ĺ����꣩
    time = calculateShootTime();
    //cout<<"time = 0.0f"<<endl;
    if (time == 0.0f) {
        return float();
    }
    //С��
     //cout<<"isBig = "<<_isBig<<endl;
    if (!_isBig) {
        if (_buffBias == FRONT) {
            if(_buffSolverStruct.shootSpeedLevel)
            _paraCircle.buffPredictAngle += FRONT_BACK_PARA;
            else
            _paraCircle.buffPredictAngle += (FRONT_BACK_PARA+1.5);
            //cout << " _paraCircle.buffPredictAngle " << _paraCircle.buffPredictAngle << endl;
        } else if (_buffBias == BACK) {
            if(_buffSolverStruct.shootSpeedLevel)
            _paraCircle.buffPredictAngle -= FRONT_BACK_PARA;
            else
            _paraCircle.buffPredictAngle -= (FRONT_BACK_PARA+1.5);
            //cout<<"para"<<(FRONT_BACK_PARA+1)<<endl;
            //cout << " _paraCircle.buffPredictAngle " << _paraCircle.buffPredictAngle << endl;
        }
        return _paraCircle.buffPredictAngle;
    }
    //���
    else {
        if (_buffBias == FRONT) {
            _sineFunction.para += FRONT_BACK_SIN;
            _sineFunction.amplitude += FRONT_BACK_SIN;
            cout << " _sineFunction.para"  << _sineFunction.para << endl;
            cout << " _sineFunction.amplitude"  << _sineFunction.amplitude << endl;
        } else if (_buffBias == BACK) {
            _sineFunction.para -= FRONT_BACK_SIN;
            _sineFunction.amplitude -= FRONT_BACK_SIN;
            cout << " _sineFunction.para"  << _sineFunction.para << endl;
            cout << " _sineFunction.amplitude"  << _sineFunction.amplitude << endl;
        }
//        cout << " _sineFunction.para"  << _sineFunction.para << endl;
//        cout << " _sineFunction.amplitude"  << _sineFunction.amplitude << endl;

        //cout<<"shoot"<<_buffSolverStruct.shootSpeedLevel<<endl;
        if(!_buffSolverStruct.shootSpeedLevel){
            _delayTime = DELAY_TIME+0.25f;
            //cout<<"delay"<<_delayTime<<endl;
        }
        //���ֳ��ı�ĽǶȣ���ʱ����л���
        _realAddAngle = (_sineFunction.amplitude / _sineFunction.rotateIndex
                         * (cosf(_sineFunction.rotateIndex * time) - cosf(_sineFunction.rotateIndex * (time + _delayTime))) + _sineFunction.para * DELAY_TIME)
                        * 180 / Util::PI_F() + _para;
        //_realAddAngle = 30;
        //_realAddAngle = _realAddAngle*0.8;
        //cout << _para << endl<<endl<<endl;     
        return _realAddAngle;
    }
}

void BuffCoordinateSolver::calculateRotateSpeed() {
//���徲̬��ȥ�����ڽǶȣ�
    static double nowAngle = 0.0f;
    static double lastAngle = 0.0f;
    static int count = 0;
    //�����ȥ������ʱ��
    static double lastTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000; // ms
    double curTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000;
    //���ҶƬû�����䣬��ѹ�ȥ�����ڽǶ��Լ���ȥ�������ٶ�����

    if (!_buffSolverStruct.sameTargetFlag) {
        lastAngle = nowAngle = _rotateSpeed.lastRotateSpeed = _rotateSpeed.nowRotateSpeed = 0.0f;
        return;
    }
    //�����ȥ�Ƕ��Ѿ������㣬���ȥ�ǶȽ��г�ʼ��Ϊ���ھ��ԽǶ�
    if (lastAngle == 0.0f) {
        lastAngle = _circleAngle360;
        return;
    }
    //ÿ0.1sһ������ˢ��
    if (curTime - lastTime < 100) {
        return;
    }
    //֡������
    count++;
    nowAngle = _circleAngle360;
    //����ʵʱ���ٶ�
    _rotateSpeed.nowRotateSpeed = (float) fabs(Util::angleToRadian((nowAngle - lastAngle)) * (1000.0f / (curTime - lastTime)));
    //��ȥ�ǶȺ�ʱ�����
    lastAngle = nowAngle;
    lastTime = curTime;
    //�����ȥ���ٶ��ѱ����㣬��Թ�ȥ�ٶȽ��и���
    if (_rotateSpeed.lastRotateSpeed == 0.0f) {
        _rotateSpeed.lastRotateSpeed = _rotateSpeed.nowRotateSpeed;
        return;
    }
    //��ֹ�����쳣����
    if (_rotateSpeed.nowRotateSpeed > 5 || _rotateSpeed.nowRotateSpeed < -5) {
        return;
    }
    //����ٶ�û���滻��С�ٶȣ��������1
    if (_speedRange.nowMinSpeed > _rotateSpeed.nowRotateSpeed) {
        _speedRange.nowMinSpeed = _rotateSpeed.nowRotateSpeed;
    } else {
        _speedRange.minSameNumber++;
    }
    //����ٶ�û���滻����ٶȣ��������1
    if (_speedRange.nowMaxSpeed < _rotateSpeed.nowRotateSpeed) {
        _speedRange.nowMaxSpeed = _rotateSpeed.nowRotateSpeed;
    } else {
        _speedRange.maxSameNumber++;
    }
    //�������20֡û��ˢ����С�ٶȣ�����ٶ�Ϊ�����ٶȣ����ٶ�һ�����£��㲻�ٸ��£�
    if (_speedRange.minSameNumber > 20 && !_speedRange.minSpeedFlag) {
        _speedRange.realMinSpeed = _speedRange.nowMinSpeed;
        _speedRange.minSpeedFlag = true;
    }
    //�������20֡û��ˢ������ٶȣ�����ٶ�Ϊ�����ٶȣ����ٶ�һ�����£��㲻�ٸ��£�
    if (_speedRange.maxSameNumber > 20 && !_speedRange.maxSpeedFlag) {
        _speedRange.realMaxSpeed = _speedRange.nowMaxSpeed;
        _speedRange.maxSpeedFlag = true;
    }
   // cout<<"update"<<_speedRange.minSpeedFlag <<" "<<_speedRange.maxSpeedFlag<<endl;
    //��ʱ�������Һ�����������������������ٶȡ�ƫ������//��Ϊ�ڲ�ͬ�ĽǶȼ�������ٶȲ�ͬ�����º����Ļ������������˸ı�
//    if (_speedRange.minSpeedFlag && _speedRange.maxSpeedFlag) {
//        _sineFunction.para = (_speedRange.realMaxSpeed + _speedRange.realMinSpeed) / 2;
//        _sineFunction.amplitude = _speedRange.realMaxSpeed - _sineFunction.para;
//        //cout<<"para"<<_sineFunction.para<<endl;
//        //cout<<"amplitude"<<_sineFunction.amplitude<<endl;
//    }
    //��ֵ��ʵ�ٶȣ��������ʹ��
    _rotateSpeed.realRotateSpeed = _rotateSpeed.nowRotateSpeed;
    _rotateSpeed.speedType = (_rotateSpeed.nowRotateSpeed > _rotateSpeed.lastRotateSpeed ? SPEED_UP : SPEED_DOWN);
}

float BuffCoordinateSolver::calculateShootTime() {
    if (_rotateSpeed.realRotateSpeed <= 0.0f) {
        return float();
    }
    //spd=0.785*sin(1.884*t)+1.305
    float possibleTime[2];
    float realTime;
    //�ٶȳ���Ҫ��
    if (_rotateSpeed.realRotateSpeed < _sineFunction.para - _sineFunction.amplitude) {
        _rotateSpeed.realRotateSpeed = _sineFunction.para - _sineFunction.amplitude;
    }
    if (_rotateSpeed.realRotateSpeed > _sineFunction.para + _sineFunction.amplitude) {
        _rotateSpeed.realRotateSpeed = _sineFunction.para + _sineFunction.amplitude;
    }
    possibleTime[0] = (asinf((_rotateSpeed.realRotateSpeed - _sineFunction.para) / _sineFunction.amplitude)) / _sineFunction.rotateIndex;
    possibleTime[1] = (possibleTime[0] > 0 ? Util::PI_F() / (_sineFunction.rotateIndex) - possibleTime[0] : Util::PI_F() / (-_sineFunction.rotateIndex) - possibleTime[0]);
   // cout << "    " << _rotateSpeed.speedType << endl;
    realTime = (_rotateSpeed.speedType == SPEED_UP ? possibleTime[0] : possibleTime[1]);
    //cout << "time--" << fabs(possibleTime[0] - possibleTime[1]) << endl;
    return realTime;
}



