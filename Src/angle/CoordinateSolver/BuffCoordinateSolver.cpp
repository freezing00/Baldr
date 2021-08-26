#include "BuffCoordinateSolver.h"

BuffCoordinateSolver::BuffCoordinateSolver(ArmorRealData armorRealData, CameraCalibrationStruct &calibrationData) : ArmorCoordinateSolver(armorRealData, calibrationData) {

}

//神符预判流程函数
ArmorCoordinateSolver::CoordinateStruct BuffCoordinateSolver::calculateWorldCoordinate(cv::Point2f * verticesToDraw, float pitchAngle, float yawAngle, CarType carType) {
   _pitchAngle = pitchAngle;
    _yawAngle = yawAngle;

    //预判
    makeFinalVertices(verticesToDraw);
    //位姿解算
    ArmorCoordinateSolver::realCoordinateCalculate(verticesToDraw, carType);
    //真实坐标解算
    ArmorCoordinateSolver::actualCoordinateSolver();
    //滤波以及数据装填
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

//预判前的初始化
void BuffCoordinateSolver::buffCoordinateCalculateInit() {
    //cout<<"init"<<endl;
    static float lastCircleAngle = 0.0f;
    static cv::Point2f lastTargetRectCenter = cv::Point2f(0.0f, 0.0f);
    static float circleAngleBias = 0.0f;
    _circleAngle180 = Util::getAngle(_buffSolverStruct.targetRect.center, cv::Point2f(3000.0f, _buffSolverStruct.R_2D_Point.y), _buffSolverStruct.R_2D_Point);
    //旋转角度处理
    if (_buffSolverStruct.R_2D_Point.y < _buffSolverStruct.targetRect.center.y) {
        _circleAngle360 = 360.0f - _circleAngle180;
        _circleAngle180 = -_circleAngle180;
    } else {
        _circleAngle360 = _circleAngle180;
    }
    //神符方向重置
    if (_resetBuff) {
        if (_buffDirOfRota != UNKNOW) {
            _buffDirOfRota = UNKNOW;
            _buffInit = false;
            _buffAngleList.resize(0);
            _resetBuff = false;
        }
    }
    circleAngleBias = _circleAngle360 - lastCircleAngle;


    //判断是否是同一片叶片
    if ((fabsf(circleAngleBias) > 10.0f || (fabsf(lastTargetRectCenter.y - _buffSolverStruct.targetRect.center.y) > 40.0f) || (fabsf(lastTargetRectCenter.x - _buffSolverStruct.targetRect.center.x) > 40.0f))) {
        _sameTargetFlag = false;
    }
    else {
        _sameTargetFlag = true;
    }


    if (!_buffInit) {
        //过去角度处理
        if (!_sameTargetFlag) {
            _buffAngleList.resize(0);//角度变化过大认为叶片跳变
        } else {
            _buffAngleList.push_back(_circleAngle360);
        }
        //数据足够判断方向

        if (_buffAngleList.size() > 5) {//存五个数据用来判断
            float averageAngle = 0.0f;
            float buff = 0.0f;
            uint8_t count = 0;
            //逐差法计算
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
            //判断旋转方向
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

//二维预判
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

        //操作手外部按键调参接口
        if (_buffBias == UP) {
            _buffArmor.length += index * _armorRatio;
            _buffArmor.width += index * _armorRatio;
//            cout << "_buffArmor.length" << _buffArmor.length << "," << "_buffArmor.width" << _buffArmor.width << endl;
        } else if (_buffBias == DOWN) {
            _buffArmor.length -= index * _armorRatio;
            _buffArmor.width -= index * _armorRatio;
//            cout << "_buffArmor.length" << _buffArmor.length << "," << "_buffArmor.width" << _buffArmor.width << endl;
        }

        //位置预判
        r = Util::pointDistance(_buffSolverStruct.R_2D_Point, _buffSolverStruct.targetRect.center)*1.08;
        _predictCoordinate.x = _buffSolverStruct.R_2D_Point.x + r * cosf(resAngle * Util::PI_F() / 180.0f);
        _predictCoordinate.y = _buffSolverStruct.R_2D_Point.y + r * sinf(resAngle * Util::PI_F() / 180.0f);
    } else {
        _predictCoordinate = _buffSolverStruct.targetRect.center;
    }
    //cout<<"prediction"<<_buffSolverStruct.targetRect.center<<endl;
    predictBias= _predictCoordinate -_buffSolverStruct.targetRect.center;

    //为在图像上呈现预判点存储数据
    for (int i = 0; i < 4; i++) {
        vertices[i] = vertices[i] + predictBias;
    }
}

float BuffCoordinateSolver::calculateAddAngle() {
    float time = 0.0f;
    //计算实时角速度
    calculateRotateSpeed();
    //计算积分开始时刻（该时刻作为角速度函数的横坐标）
    time = calculateShootTime();
    //cout<<"time = 0.0f"<<endl;
    if (time == 0.0f) {
        return float();
    }
    //小符
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
    //大符
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
        //积分出改变的角度，对时间进行积分
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
//定义静态过去和现在角度；
    static double nowAngle = 0.0f;
    static double lastAngle = 0.0f;
    static int count = 0;
    //定义过去和现在时间
    static double lastTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000; // ms
    double curTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000;
    //如果叶片没有跳变，则把过去和现在角度以及过去和现在速度置零

    if (!_buffSolverStruct.sameTargetFlag) {
        lastAngle = nowAngle = _rotateSpeed.lastRotateSpeed = _rotateSpeed.nowRotateSpeed = 0.0f;
        return;
    }
    //如果过去角度已经被清零，则过去角度进行初始化为现在绝对角度
    if (lastAngle == 0.0f) {
        lastAngle = _circleAngle360;
        return;
    }
    //每0.1s一次数据刷新
    if (curTime - lastTime < 100) {
        return;
    }
    //帧数递增
    count++;
    nowAngle = _circleAngle360;
    //计算实时角速度
    _rotateSpeed.nowRotateSpeed = (float) fabs(Util::angleToRadian((nowAngle - lastAngle)) * (1000.0f / (curTime - lastTime)));
    //过去角度和时间更新
    lastAngle = nowAngle;
    lastTime = curTime;
    //如果过去角速度已被清零，则对过去速度进行更新
    if (_rotateSpeed.lastRotateSpeed == 0.0f) {
        _rotateSpeed.lastRotateSpeed = _rotateSpeed.nowRotateSpeed;
        return;
    }
    //防止出现异常数据
    if (_rotateSpeed.nowRotateSpeed > 5 || _rotateSpeed.nowRotateSpeed < -5) {
        return;
    }
    //如果速度没有替换最小速度，则计数加1
    if (_speedRange.nowMinSpeed > _rotateSpeed.nowRotateSpeed) {
        _speedRange.nowMinSpeed = _rotateSpeed.nowRotateSpeed;
    } else {
        _speedRange.minSameNumber++;
    }
    //如果速度没有替换最大速度，则计数加1
    if (_speedRange.nowMaxSpeed < _rotateSpeed.nowRotateSpeed) {
        _speedRange.nowMaxSpeed = _rotateSpeed.nowRotateSpeed;
    } else {
        _speedRange.maxSameNumber++;
    }
    //如果连续20帧没有刷新最小速度，则该速度为波谷速度（该速度一旦更新，便不再更新）
    if (_speedRange.minSameNumber > 20 && !_speedRange.minSpeedFlag) {
        _speedRange.realMinSpeed = _speedRange.nowMinSpeed;
        _speedRange.minSpeedFlag = true;
    }
    //如果连续20帧没有刷新最大速度，则该速度为波峰速度（该速度一旦更新，便不再更新）
    if (_speedRange.maxSameNumber > 20 && !_speedRange.maxSpeedFlag) {
        _speedRange.realMaxSpeed = _speedRange.nowMaxSpeed;
        _speedRange.maxSpeedFlag = true;
    }
   // cout<<"update"<<_speedRange.minSpeedFlag <<" "<<_speedRange.maxSpeedFlag<<endl;
    //此时更新正弦函数的三个参数（振幅、角速度、偏移量）//因为在不同的角度计算出的速度不同，导致函数的基本参数发生了改变
//    if (_speedRange.minSpeedFlag && _speedRange.maxSpeedFlag) {
//        _sineFunction.para = (_speedRange.realMaxSpeed + _speedRange.realMinSpeed) / 2;
//        _sineFunction.amplitude = _speedRange.realMaxSpeed - _sineFunction.para;
//        //cout<<"para"<<_sineFunction.para<<endl;
//        //cout<<"amplitude"<<_sineFunction.amplitude<<endl;
//    }
    //赋值真实速度，方便后面使用
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
    //速度超限要改
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



