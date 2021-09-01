#include "angleFactory.h"
#include "KalmanPredict/KalmanPredict.h"

AngleFactory::AngleFactory(const string& serialNumber) {
    //_buffTargetNoChangeFlag = true;

    //相机内参信息,畸变系数矩阵读取
    CameraCalibrationStruct calibrationData = CCalibration::readCalibrationData(CALI_RESULTS_PATH + serialNumber + ".yml");
    CCalibration::printCalibrationData(calibrationData);
    //小装甲板实际尺寸填装，单位毫米
    _smallArmorCoordinateSolver = new ArmorCoordinateSolver(ArmorDataFactory::getSmallArmor(), calibrationData);
    //大装甲板实际尺寸填装，单位毫米
    _bigArmorCoordinateSolver = new ArmorCoordinateSolver(ArmorDataFactory::getBigArmor(), calibrationData);
    //神符装甲板实际尺寸填装，单位毫米
    _buffCoordinateSolver = new BuffCoordinateSolver(ArmorDataFactory::getBuffArmor(),calibrationData);
}

void AngleFactory::calculateFinalResult(const cv::RotatedRect &targetRect, ArmorType armorType, DistinguishMode distinguishMode, CarType carType, float pitchAngle, float yawAngle,float shootSpeed,bool sameTargetFlag,int targetWidth, bool topFlag) {

    static cv::Point2f vertices[4];
    static cv::Point2f verticesCal[4];
    targetRect.points(verticesCal);
    _carType = carType;

    if (distinguishMode == TX2_DISTINGUISH_BUFF || distinguishMode == TX2_DISTINGUISH_BIG_BUFF) {
        if (targetRect.size.width < targetRect.size.height) {
            for (int i = 0; i < 4; i++) {
                vertices[i] = verticesCal[(i + 1) % 4];
            }
        }
        else {
            for (int i = 0; i < 4; i++) {
                vertices[i] = verticesCal[i];
            }
        }
    }
    else {
        if (targetRect.angle < -90.0f) {
            for (int i = 0; i < 4; i++) {
                vertices[i] = verticesCal[(i + 3) % 4];
            }
        }
        else {
            for (int i = 0; i < 4; i++) {
                vertices[i] = verticesCal[(i + 1) % 4];
            }
        }
    }

    //    for(int i = 0; i < 4; i++)
    //        std::cout << vertices[i] << std::endl;
    //    std::cout << std::endl;

    switch (distinguishMode) {
    case TX2_STOP:
        break;
    case TX2_DISTINGUISH_ARMOR:

        if (armorType == ARMOR_SMALL) {
            _smallArmorCoordinateSolver->setLastDistinguishMode(distinguishMode);
            _coordinateStruct = _smallArmorCoordinateSolver->calculateWorldCoordinate(vertices, pitchAngle, yawAngle, shootSpeed, carType,targetWidth, topFlag);
        }
        else {
            _bigArmorCoordinateSolver->setLastDistinguishMode(distinguishMode);
            _coordinateStruct = _bigArmorCoordinateSolver->calculateWorldCoordinate(vertices, pitchAngle, yawAngle, shootSpeed, carType, targetWidth, topFlag);
        }
        break;

    case TX2_DISTINGUISH_BUFF:
    case TX2_DISTINGUISH_BIG_BUFF:
        _buffCoordinateSolver->setLastDistinguishMode(distinguishMode);
        _buffCoordinateStruct = _buffCoordinateSolver->calculateWorldCoordinate(vertices, pitchAngle, yawAngle, carType);
        _coordinateStruct.actualCoordinate = _buffCoordinateStruct.actualCoordinate;
        _buffTargetNoChangeFlag = _buffCoordinateSolver->getSameTargetFlag();
        break;
    }
}

//函数模型为斜抛运动，弹丸从枪口以一定初速度射出，命中远处的目标
void AngleFactory::ballisticParabolaFunCalculation(float resRadian, float* funtionValue, float* derivativeValue, float p, float h, float v) {
    float sinResAngle = (float)sin(resRadian);
    float cosResAngle = (float)cos(resRadian);
    float tanResAngle = (float)(sinResAngle / (cosResAngle + TINY));
    float v_pow = v * v;
    float p_pow = p * p;
    float cosRes_pow = cosResAngle * cosResAngle;

//    cout << "sinResAngle = " << sinResAngle << endl;
//    cout << "cosResAngle = " << cosResAngle << endl;
//    cout << "tanResAngle = " << tanResAngle << endl;
//    cout << "cosRes_pow  = " << cosRes_pow << endl;

    //函数值为当前迭代角度带入公式时，弹丸在到达P距离时在垂直距离上与目标的差值（我们希望这个值趋近于0，使得弹丸正中目标）
    *funtionValue = tanResAngle * p - 0.5f * GRAVITY * p_pow / (v_pow * cosRes_pow + TINY) - h;
    //对该函数求导，得到下式
    *derivativeValue = p / (cosRes_pow + TINY) + (GRAVITY * p_pow / (v_pow + TINY)) * tanResAngle / (cosRes_pow + TINY);
}

//使用牛顿迭代法迭代得到使弹丸能抵消重力影响正中目标的Pitch轴角度值
float AngleFactory::newtonIterationMethod(Stm32CmdStruct Stm32Cmd) {

    //单位转化，这里的单位是m
    float y = (-MM_TO_M_TRANS(_coordinateStruct.worldOriginInPZT.y));
    float z = (MM_TO_M_TRANS(_coordinateStruct.worldOriginInPZT.z));

    //cout << "y = " << y << endl;
    //cout << "z = " << z << endl;

    float resPitchAngle = Stm32Cmd.pitchData.float_temp - (float)atan(-y / z) * 180.0f / PI;    //期望角度赋初值，后面要不断迭代来抵消重力影响
    float funtionValue;
    float derivativeValue;
    unsigned int cnt = 0;
    float temp = 1.0f;  //牛顿迭代法中每次迭代减去的值，我们在这姑且叫它“余项”

    float v = Stm32Cmd.shootSpeed;

    float alpha = Stm32Cmd.pitchData.float_temp;    //α以向上旋转为正值，为云台Pitch轴当前角度（水平为0度）

    float beta = (float)atan(y / z) * 180.0f / PI;  //β以向上旋转为正值，为云台指向与目标位置之间的夹角

    float gamma = alpha + beta;                     //γ以向上旋转为正值，为目标位置与水平线之间的夹角

    //cout << "alpha = " << alpha << endl;
    //cout << "beta  = " << beta << endl;
    //cout << "gamma = " << gamma << endl;

    float sinBeta = (float)sin(ANGLE_TO_RADIAN(beta));
    float cosBeta = (float)cos(ANGLE_TO_RADIAN(beta));
    float sinGamma = (float)sin(ANGLE_TO_RADIAN(gamma));
    float cosGamma = (float)cos(ANGLE_TO_RADIAN(gamma));

    //cout << "sinBeta  = " << sinBeta << endl;
    //cout << "cosBeta  = " << cosBeta << endl;
    //cout << "sinGamma = " << sinGamma << endl;
    //cout << "cosGamma = " << cosGamma << endl;

    float p = (z / (cosBeta + TINY)) * cosGamma;
    float h = (z / (cosBeta + TINY)) * sinGamma;

    //cout << "p = " << p << endl;
    //cout << "h = " << h << endl;
    //cout << "v = " << v << endl;

    while ((fabsf(temp) > 1e-3f) && (cnt < 400)) {

        ballisticParabolaFunCalculation(ANGLE_TO_RADIAN(resPitchAngle), &funtionValue, &derivativeValue, p, h, v);

        if ((derivativeValue != 0) && (!isnan(derivativeValue)) && (!isnan(funtionValue)) && (derivativeValue < 1e+5f) && (funtionValue < 1e+5f)) {
            temp = funtionValue / (derivativeValue + TINY); //余项等于函数值与函数导数值的比，参考牛顿迭代法公式

            if (temp < 70) {
                resPitchAngle -= temp;  //resPitchAngle 减去余项，Pitch轴角度不断迭代，最后得到可以抵消重力影响命中目标的角度

                //cout << "delta = " << temp << endl;
                //cout << "now pitch = " << resPitchAngle << endl;
            }
        }
        cnt++;
        //cout << endl;
    }
    //cout << "total cnt = " << cnt << endl;

    return resPitchAngle;
}

float AngleFactory::equationCalculation(float shootSpeed, float pitchAngle, cv::Point3f coordinateActual) {
    float pitchAngleRef = pitchAngle;
    //相机坐标系旋转，使其zox平面水平,y轴垂直 
    float x = MM_TO_M_TRANS(coordinateActual.x);
    float z = MM_TO_M_TRANS(coordinateActual.z);
    float y = -MM_TO_M_TRANS(coordinateActual.y);	//垂直距离，该坐标系y轴正方向为向下，故取反

    float speed = shootSpeed;
    float tanAngleA = 0.0f;
    float tanAngleB = 0.0f;

    float p = sqrtf(x * x + z * z);     //获取水平距离
    //cout << "horizonPosition = " << p << endl;
    //cout << "verticalPosition = " << y << endl;
    //cout << "shootSpedd = " << shootSpeed << endl;

    float a = -GRAVITY * p * p / (2 * speed * speed);
    float b = p;
    float c = (-y + a);
    float delta = b * b - 4 * a * c;
    float mid = -b / (2 * a);
    /*
    cout << "a = " << a << endl;
    cout << "b = " << b << endl;
    cout << "c = " << c << endl;
    cout << "delta = " << delta << endl;
    cout << "mid = " << mid << endl;
    */
    if (delta < 0) {
        pitchAngleRef = pitchAngle;
    }
    else if (delta == 0) {
        tanAngleA = mid;
        pitchAngleRef = atanf(tanAngleA) * 180 / PI;
        //cout << "pitchAngleRef = " << pitchAngleRef << endl;
    }
    else if (delta > 0) {

        tanAngleA = mid - sqrtf(delta) / (2 * a);
        tanAngleB = mid + sqrtf(delta) / (2 * a);

        if (tanAngleA >= -1 && tanAngleA <= 1) {
            pitchAngleRef = atanf(tanAngleA) * 180 / PI;
        }
        else if (tanAngleB >= -1 && tanAngleB <= 1) {
            pitchAngleRef = atanf(tanAngleB) * 180 / PI;
        }
        /*
        cout << "tanAngleA = " << tanAngleA << endl;
        cout << "tanAngleB = " << tanAngleB << endl;
        cout << "pitchAngleRef = " << pitchAngleRef << endl;
        */
    }
    return pitchAngleRef;
}

float AngleFactory::airResistCalculation(float shootSpeed, float pitchAngle, cv::Point3f coordinateActual) {

    float pitchAngleRef = pitchAngle;
    float KAPPA = 0.00045f; //大弹丸的空气阻力系数
    float M = 0.04f;        //大弹丸的质量

    float x = MM_TO_M_TRANS(coordinateActual.x);
    float z = MM_TO_M_TRANS(coordinateActual.z);
    float y = -MM_TO_M_TRANS(coordinateActual.y);
    float speed = shootSpeed;
    float p = sqrt(x * x + z * z);
    float A = sqrt(1 - 2 * p * KAPPA / M);

    /*
    cout << "shootSpeed = " << speed << endl;
    cout << "horizonPosition = " << p << endl;
    cout << "verticalPosition = " << y << endl;
    */

    float a = -0.5 * GRAVITY * (M * (1 - A) / (KAPPA * speed)) * (M * (1 - A) / (KAPPA * speed));
    float b = M * (1 - A) / KAPPA;
    float c = a - y;
    float delta = b * b - 4 * a * c;
    float mid = -b / (2 * a);
    float tanAngleA = 0;
    float angleA = 0;
    float tanAngleB = 0;
    float angleB = 0;
    /*
    cout << "a = " << a << endl;
    cout << "b = " << b << endl;
    cout << "c = " << c << endl;
    cout << "delta = " << delta << endl;
    cout << "mid = " << mid << endl;
    */
    if (delta < 0) {
        pitchAngleRef = pitchAngle;
    }
    else if (delta == 0) {
        tanAngleA = mid;
        pitchAngleRef = atan(tanAngleA) * 180 / PI;
        //cout << "pitchAngleRef = " << pitchAngleRef << endl;
    }
    else if (delta > 0) {

        tanAngleA = mid - sqrt(delta) / (2 * a);
        angleA = atanf(tanAngleA) * 180 / PI;;
        tanAngleB = mid + sqrt(delta) / (2 * a);
        angleB = atan(tanAngleB) * 180 / PI;

        if (tanAngleA >= -1 && tanAngleA <= 1) {
            pitchAngleRef = atanf(tanAngleA) * 180 / PI;
        }
        else if (tanAngleB >= -1 && tanAngleB <= 1) {
            pitchAngleRef = atan(tanAngleB) * 180 / PI;
        }

        //cout << "tanAngleA = " << tanAngleA << endl;
        //cout << "tanAngleB = " << tanAngleB << endl;
        //cout << "pitchAngleRef = " << pitchAngleRef << endl;

    }
    return pitchAngleRef;
}

float AngleFactory::getYawAngle(float x, float z, float angle) {
    //固有坐标系，伸出右手呈右手坐标系状，中指朝下，拇指朝前。拇指为z轴，食指指为x轴，中指为y轴
    //陀螺仪返回角度，顺时针为正角

    /*******特殊情况判断******/
    //不合法坐标，返回NAN
    if (x == 0 && z == 0 || isnan(x) || isnan(z)) {
        // LOG::debug("getYawAngle coordinate is nan!");
        return INFINITY;
    }
    //角度正在0度，直接返回 
    if (angle == 0) return 0.0f;

    //在x轴上的情况
    if (z == 0) {
        //角度在正角区
        if (angle > 0) {
            if (x > 0)  return 90.0f;
            else        return 270.0f;
        }
        //角度在负角区
        else {
            if (x > 0)  return -270.0f;
            else        return -90.0f;
        }
    }
    //在z轴上的情况
    if (x == 0) {
        if (angle > 0) {
            if (z > 0)  return 0.0f;
            else        return 180.0f;
        }
        //角度在负角区
        else {
            if (z > 0)  return 0.0f;
            else        return -180.0f;
        }
    }

    int cnt = angle / 360;
    float bias = angle - 360 * cnt;
    //cout << "cnt: " << cnt << endl;
    //cout << "bias: " << bias << endl;

    float yawAngle = bias;
    //在正角区 
    if (angle > 0) {
        //第一象限
        if (x > 0 && z > 0) {
            yawAngle = atanf(x / z) * 180 / PI;
        }
        //第二象限
        else if (x < 0 && z > 0) {
            yawAngle = 360 - atanf(-x / z) * 180 / PI;
        }
        //第三象限
        else if (x < 0 && z < 0) {
            yawAngle = 180 + atanf(x / z) * 180 / PI;
        }
        //第四象限
        else if (x > 0 && z < 0) {
            yawAngle = 180 - atanf(x / -z) * 180 / PI;
        }
        else return INFINITY;
    }
    //在负角区
    else {
        //第一象限
        if (x > 0 && z > 0) {
            yawAngle = atanf(x / z) * 180 / PI - 360;
        }
        //第二象限
        else if (x < 0 && z > 0) {
            yawAngle = -atanf(-x / z) * 180 / PI;
        }
        //第三象限
        else if (x < 0 && z < 0) {
            yawAngle = atanf(x / z) * 180 / PI - 180;
        }
        //第四象限
        else if (x > 0 && z < 0) {
            yawAngle = -180 - atanf(x / -z) * 180 / PI;
        }
        else return INFINITY;
    }
    //cout << "before: " << yawAngle << endl;
    yawAngle += 360 * cnt;

    if (fabsf(yawAngle - angle) > 180) {
        if (angle > 0) {
            yawAngle -= 360;
        }
        else {
            yawAngle += 360;
        }
    }
    //cout << "after: " << yawAngle << endl;
    return yawAngle;
}

void AngleFactory::getAngleCalculateData(TX2CmdStruct* TX2Cmd, Stm32CmdStruct Stm32Cmd) {

    ArmorCoordinateSolver::CoordinateStruct coordinatePredict;
    coordinatePredict = _coordinateStruct;

    static float lastYawData = Stm32Cmd.yawData.float_temp;
    static float lastPitchData = Stm32Cmd.pitchData.float_temp;
    static float lastYawRef = Stm32Cmd.yawData.float_temp;
    static float lastPitchRef = Stm32Cmd.pitchData.float_temp;

    static float nowTime_ms = 0.0f, deltaTime_ms = 0.0f, lastTime_ms = 0.0f;
    nowTime_ms = cv::getTickCount() / cv::getTickFrequency() * 1000;	//ms
    deltaTime_ms = (float)(nowTime_ms - lastTime_ms);				    //ms
    lastTime_ms = nowTime_ms;

    //间隔MAX_TIME_BIAS，视为两次跟随
    if (deltaTime_ms > MAX_TIME_BIAS) {
        lastYawData = Stm32Cmd.yawData.float_temp;
        lastPitchData = Stm32Cmd.pitchData.float_temp;
        lastYawRef = Stm32Cmd.yawData.float_temp;
        lastPitchRef = Stm32Cmd.pitchData.float_temp;
    }

    //检测角度数据的合法性
    float yawData, pitchData;
    if (isnan(Stm32Cmd.yawData.float_temp)) {
        yawData = lastYawData;
        //LOG::debug("yawData is nan!");
        cout << "yawData is nan!" << endl;
    }
    else if (fabs(Stm32Cmd.yawData.float_temp - lastYawData) > 45.0f) {
        yawData = lastYawData;
        //LOG::debug("yawData is abnormal!");
        //cout << "yawData is abnormal!" << endl;
    }
    else {
        yawData = Stm32Cmd.yawData.float_temp;
        lastYawData = yawData;
    }

    if (isnan(Stm32Cmd.pitchData.float_temp)) {
        pitchData = lastPitchData;
        //LOG::debug("pitchData is nan!");
        cout << "pitchData is nan!" << endl;
    }
    else if (fabs((Stm32Cmd.pitchData.float_temp - lastPitchData) > 45.0f)) {
        pitchData = lastPitchData;
        //LOG::debug("pitchData is abnormal!");
        //cout << "pitchData is abnormal!" << endl;
    }
    else {
        pitchData = Stm32Cmd.pitchData.float_temp;
        lastPitchData = pitchData;
    }

    //获得处理后的陀螺仪角度期望
    float yawResult = getYawAngle(coordinatePredict.actualCoordinate.x, coordinatePredict.actualCoordinate.z, yawData);
    float pitchResult;
    //英雄大弹丸要考虑空气阻力
    if (_carType == OLD_HERO || _carType == NEW_HERO) {
        pitchResult = airResistCalculation(Stm32Cmd.shootSpeed, pitchData, coordinatePredict.actualCoordinate);
    }
    else {
        pitchResult = equationCalculation(Stm32Cmd.shootSpeed, pitchData, coordinatePredict.actualCoordinate);
    }

    //筛除nan值
    if (isnan(yawResult)) {
        yawResult = lastYawRef;    //出现异常
        //LOG::debug("yawResult is nan!");
        cout << "yawResult is nan!" << endl;
    }
    if (isnan(pitchResult)) {
        pitchResult = lastPitchRef;
        //LOG::debug("pitchResult is nan!");
        cout << "pitchResult is nan!" << endl;
    }

    //跳变检查
    if (fabs(yawResult - yawData) >= 90.0f && TX2Cmd->sameTargetFlag == true) {
        yawResult = yawData;
        //LOG::debug("yawResult jump too big!");
        //cout << "yawResult jump too big!" << endl;
    }
    if (fabs(pitchResult - pitchData) >= 75.0f && TX2Cmd->sameTargetFlag == true) {
        pitchResult = pitchData;
        //LOG::debug("pitchResult jump too big!");
        //cout << "pitchResult jump too big!" << endl;
    }

    //最终角度期望
    TX2Cmd->yawData.float_temp = yawResult;
    TX2Cmd->pitchData.float_temp = pitchResult;

    lastYawRef = yawResult;
    lastPitchRef = pitchResult;

    if (CurveData::isEnable) {
        //无延时补偿时的预判坐标
        cv::Point3f coordinateNoBias = CurveData::getNoBiasCoordinate();
        float yawNoBias = getYawAngle(coordinateNoBias.x, coordinateNoBias.z, yawData);
        float pitchNoBias = equationCalculation(Stm32Cmd.shootSpeed, pitchData, coordinateNoBias);
        CurveData::saveAngle(cv::Point2f(yawNoBias, pitchNoBias), cv::Point2f(yawResult, pitchResult), \
            cv::Point2f(Stm32Cmd.yawData.float_temp, Stm32Cmd.pitchData.float_temp));
    }

//    cout << "yawBias:" << TX2Cmd->yawData.float_temp - Stm32Cmd.yawData.float_temp << endl;
//    cout << "yawRef:" << TX2Cmd->yawData.float_temp << endl;
//    cout << "yawFbd: " << Stm32Cmd.yawData.float_temp << endl;
//      cout << "pitchRef:" << TX2Cmd->pitchData.float_temp << endl;
//      cout << "pitchFbd" << Stm32Cmd.pitchData.float_temp << endl << endl;
}


void AngleFactory::resetAngleCalculateData(TX2CmdStruct* TX2Cmd) {

    TX2Cmd->sameTargetFlag = false;
    TX2Cmd->getOrderFlag = false;
    TX2Cmd->captureCmd = false;
}

bool AngleFactory::getBuffTargetNoChangeFlag() {
    return _buffTargetNoChangeFlag;
}


bool AngleFactory::_buffTargetNoChangeFlag;//静态成员定义
