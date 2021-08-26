#include <angle/KalmanPredict/KalmanPredict.h>
#include "angle/angleFactory.h"

void KalmanFilter::Prediction() {
	_x = _f * _x;	//+ U_;	
	//std::cout << "_x: " << _x << std::endl << std::endl;
	//std::cout << "_f: " << _f << std::endl << std::endl;
	_p = _f * _p * _f.transpose() + _q;					//��һʱ��״̬Э��������� ��һʱ�̵��Լ��任�������ټ������� 
}

void KalmanFilter::MeasurementUpdate(const Eigen::VectorXd& z) {
	Eigen::VectorXd y = z - _h * _x;						//��ȡ����ֵ�� Ԥ�����ֵ֮��Ĳ�ֵ
	Eigen::MatrixXd S = _h * _p * _h.transpose() + _r;		//��ʱ����
	Eigen::MatrixXd K = _p * _h.transpose() * S.inverse();	//��ȡ����������
	_x = _x + (K * y);										//���״̬���Ź���
	//std::cout << "y: " << y << std::endl << std::endl;
	int size = _x.size();
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
	_p = (I - K * _h) * _p;									//����״̬Э�������
}

cv::Point3f KalmanFilter::KFilter(cv::Point3f data) {

	float deltaTime_s = _deltaTime_ms / 1000.0f;
	float x = data.x, y = data.y, z = data.z;		
	Eigen::VectorXd inputX(6, 1);
	inputX << x, y, z, 0, 0, 0;		

	Eigen::MatrixXd inputF(6, 6);		//��ʼ��״̬ת�ƾ���

	inputF <<	1.0, 0.0, 0.0, deltaTime_s, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0, deltaTime_s, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0, deltaTime_s,
				0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

	SetF(inputF);

	if (!isInitialized()) {
		Initialization(inputX);
	}

	Prediction();
	Eigen::VectorXd h(3, 1);	//���ι۲�ֵ
	h << x, y, z;
	MeasurementUpdate(h);

	Eigen::VectorXd xout = GetX();

	cv::Point3f result;
	result.x = xout(0);
	result.y = xout(1);
	result.z = xout(2);

	return result;
}

cv::Point3f KalmanFilter::KPredictionRun(cv::Point3f originData) {

	_nowTime_ms = cv::getTickCount() / cv::getTickFrequency() * 1000;	//ms
	_deltaTime_ms = (float)(_nowTime_ms - _lastTime_ms);				//ms
	_lastTime_ms = _nowTime_ms;

    cv::Point3f data = originData;
    cv::Point3f dataResult = data;

	static cv::Point3f lastDataResult = originData;

	//�������˲�ʱ��������MAX_TIME_BIASms,��Ϊ���θ���
	if (_deltaTime_ms > MAX_TIME_BIAS) {
        data = originData;
		_lastOriginData = data;
		dataResult = data;
		lastDataResult = data;
		//LOG::debug("Filter _deltaTime_ms is " + std::to_string(_deltaTime_ms) + " ms! timed out! ");
        _initFlag = false;
		return dataResult;	//���ص�ǰ����
	}
	else {
        data = wrongDataKiller(originData);
		_lastOriginData = data;
		//����װ�ؿ������˲��������
		cv::Point3f dataK = data;
		dataK = KFilter(dataK);

		if (isnan(dataK.x) || isnan(dataK.y) || isnan(dataK.z)) {
			//LOG::debug("filter data is nan! ");
			cout << "filter data is nan! " << endl;
			dataResult = lastDataResult;
			_initFlag = false;		//�����쳣���������˲�����
		}
		else {
			dataResult = dataK;
			lastDataResult = dataResult;
			_initFlag = true;
		}

		return dataResult;	//���ص�ǰ����
	}
}

cv::Point3f KalmanFilter::wrongDataKiller(cv::Point3f originData) {
	//���ڷǷ�����
	if (isnan(originData.x) || isnan(originData.y) || isnan(originData.z)) {
        //LOG::debug("Filter originData.x is nan! ");
        cout << "Filter originData.x is nan! " << endl;
		return _lastOriginData;
	}

	//�޳�����������
    if (fabs(originData.x) >= _maxLimit.x || fabs(originData.y) >= _maxLimit.y  || fabs(originData.z) >= _maxLimit.z ) {
        //LOG::debug("Filter originData.x is abnormal! ");
        cout << "Filter originData.x is abnormal! " << endl;
        return _lastOriginData;
	}

	cv::Point3f result = originData;
	//�޳�����
	bool badFlagC = false;		//����ͻ���־λ
	//�޷��˲�
	//�������ɸ����������
    if (_badCntC < 5) {
        if (fabs(originData.x - _lastOriginData.x) > _jumpLimit.x) {
            //LOG::debug("Filter originData.x Jump! ");
            //cout << "Filter originData.x Jump! " << endl;
            //cout << "originData.x: " << originData.x << endl;
            //cout << "_lastOriginData.x: " << _lastOriginData.x << endl;

			badFlagC = true;
			result.x = _lastOriginData.x;
		}
		else {
			result.x = originData.x;
		}

        if (fabs(originData.y - _lastOriginData.y) > _jumpLimit.y) {
            //LOG::debug("Filter originData.y Jump! ");
            //cout << "Filter originData.y Jump! " << endl;
            //cout << "originData.y: " << originData.y << endl;
            //cout << "_lastOriginData.y: " << _lastOriginData.y << endl;

			badFlagC = true;
			result.y = _lastOriginData.y;
		}
		else {
			result.y = originData.y;
		}

        if (fabs(originData.z - _lastOriginData.z) > _jumpLimit.z) {
            //LOG::debug("Filter originData.z Jump! ");
            //cout << "Filter originData.z Jump! " << endl;
            //cout << "originData.z: " << originData.z << endl;
            //cout << "_lastOriginData.z: " << _lastOriginData.z << endl;

			badFlagC = true;
			result.z = _lastOriginData.z;
		}
		else {
			result.z = originData.z;
		}

	}
	//������γ����������ݣ������Ÿ�����
	else {
        _badCntC = 0;
        result = originData;
        badFlagC = false;
        //LOG::debug("believe Jump Filter Origin coordinate");
        //cout << "believe Jump Filter Origin coordinate" << endl;
        //cout << "originData: " << originData << endl;
	}
	//�Ƿ������������ 
	if (badFlagC == true) {
        _badCntC++;
	}
	else {
        _badCntC = 0;
	}

    //cout << "_badCntC: " << _badCntC << endl;

	return result;
}

void Prediction::getDeltaTime(float time) {
	_nowTime_ms = time;	//ms
	_deltaTime_ms = (float)(_nowTime_ms - _lastTime_ms);
	_lastTime_ms = _nowTime_ms;
}

void Prediction::subMethod( float data[][3], cv::Point3f& result ) {

	float dxArr[V_CAL_MAX_VAL / 2] = { 0 }, dyArr[V_CAL_MAX_VAL / 2] = { 0 }, dzArr[V_CAL_MAX_VAL / 2] = { 0 };	//΢����
	float vtArr[V_CAL_MAX_VAL / 2] = { 0 };																		//΢��ʱ���� ��	

	//��ȡ΢��
	for (int i = 0; i < V_CAL_MAX_VAL / 2; i++) {
		//��ȡ��Ӧ V_CAL_MAX_VAL / 2 �����ݵ�ʱ��
		for (int j = 0; j < V_CAL_MAX_VAL / 2; j++) {
			vtArr[i] += _deltaTime_s[i + j];
		}
		//cout << "vtArr" << i << ": " << tArr[i] << endl;
        dxArr[i] = ((data[i + V_CAL_MAX_VAL / 2][0]) - (data[i][0])) / vtArr[i];
        dyArr[i] = ((data[i + V_CAL_MAX_VAL / 2][1]) - (data[i][1])) / vtArr[i];
        dzArr[i] = ((data[i + V_CAL_MAX_VAL / 2][2]) - (data[i][2])) / vtArr[i];
		//cout << "dxArr" << i << ": " << dxArr[i] << endl;
		//cout << "dyArr" << i << ": " << dyArr[i] << endl;
		//cout << "dzArr" << i << ": " << dzArr[i] << endl;
	}

	//��ʼ��΢������
	result = cv::Point3f(0.0f, 0.0f, 0.0f);
	//�ݴ�΢������
	cv::Point3f dTemp = cv::Point3f(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < V_CAL_MAX_VAL / 2; i++) {
		dTemp.x += dxArr[i];
		dTemp.y += dyArr[i];
		dTemp.z += dzArr[i];
	}
	//��ȡƽ��΢��
	dTemp.x /= (float)(V_CAL_MAX_VAL / 2);
	dTemp.y /= (float)(V_CAL_MAX_VAL / 2);
	dTemp.z /= (float)(V_CAL_MAX_VAL / 2);

	result = dTemp;
	cv::Point3f origin, filter;
	origin = result;

	/*
	cout << "vx: " << _V.x << endl;
	cout << "vy: " << _V.y << endl;
	cout << "vz: " << _V.z << endl;
	*/
}

void Prediction::dataCal(cv::Point3f coordinate) {

	//���θ�����ʱ����ڣ�����Ԥ��
	if (_deltaTime_ms > MAX_TIME_BIAS) {
		_initFlag = false;
	}

	//��װ��ʼ����
	if (_initFlag == false) {
		_V = cv::Point3f(0.0f, 0.0f, 0.0f);
		_lastV = _V;
		_A = cv::Point3f(0.0f, 0.0f, 0.0f);
		_lastA = _A;

        CurveData::saveVelocity(cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));		//��װ��ʼ����ֵʱ��¼�ٶ�Ϊ0�������ٶ�������ʱ�����������Բ��ϣ��޷��������ߣ�
		CurveData::saveAcc(cv::Point3f(0, 0, 0));		

		_dataCnt = 0;
        _initFlag = true;

	}
	//�ǳ�ʼ���������ǰ�ƣ�βλ���
	else {
		//��װ V_CAL_MAX_VAL �����꣨��Ϊ��һ�����꣩
		if (_dataCnt < V_CAL_MAX_VAL) {

            _data_m[_dataCnt][0] = MM_TO_M_TRANS(coordinate.x);
            _data_m[_dataCnt][1] = MM_TO_M_TRANS(coordinate.y);
            _data_m[_dataCnt][2] = MM_TO_M_TRANS(coordinate.z);

			_velo_s[_dataCnt][0] = 0.0f;
			_velo_s[_dataCnt][1] = 0.0f;
			_velo_s[_dataCnt][2] = 0.0f;

			//��ȡ����ʱ���� 
			if(_dataCnt != 0)	
				_deltaTime_s[_dataCnt - 1] = _deltaTime_ms / 1000.0f;	//��λ ��
			//cout << "now deltaTime: " << _deltaTime_s[_dataCnt - 1]  << "s" << endl;

			CurveData::saveVelocity(cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));		//��װ��ʼ����ֵʱ��¼�ٶ�Ϊ0�������ٶ�������ʱ�����������Բ��ϣ��޷��������ߣ�
			CurveData::saveAcc(cv::Point3f(0, 0, 0));

			//�����ǰ��������
			/*
			cout << "store coordinate : x y z" << endl;
			for (int i = 0; i < V_CAL_MAX_VAL; i++) {
				for (int j = 0; j < 3; j++) {
                    cout << " " << _data_m[i][j];
				}
				cout << endl;
			}
			*/
		}
		else {

			for (int i = 0; i < V_CAL_MAX_VAL - 1; i++) {
				for (int j = 0; j < 3; j++) {
                    _data_m[i][j] = _data_m[i + 1][j];
				}
			}

            _data_m[V_CAL_MAX_VAL - 1][0] = MM_TO_M_TRANS(coordinate.x);
            _data_m[V_CAL_MAX_VAL - 1][1] = MM_TO_M_TRANS(coordinate.y);
            _data_m[V_CAL_MAX_VAL - 1][2] = MM_TO_M_TRANS(coordinate.z);

			//ʱ������ǰ��
			for (int i = 0; i < V_CAL_MAX_VAL - 2; i++) {
				_deltaTime_s[i] = _deltaTime_s[i + 1];
			}
			//��ȡ����ʱ���� 
			_deltaTime_s[V_CAL_MAX_VAL - 2] = _deltaTime_ms / 1000.0f;	//��λ ��
			//cout << "now deltaTime: " << _deltaTime_s[V_CAL_MAX_VAL - 2]  << "s" << endl;

            subMethod(_data_m, _V);	//�ٶȼ���
			cv::Point3f origin = _V;
            //cout << "originV: " << _V << endl;
			_V = _velocityFilter.KPredictionRun(_V);			//���ٶȽ��п������˲�
			CurveData::saveVelocity(origin, _V);				//��¼�˲�����ٶ�
            //cout << "filterV: " << _V << endl;
			for (int i = 0; i < V_CAL_MAX_VAL - 1; i++) {
				for (int j = 0; j < 3; j++) {
					_velo_s[i][j] = _velo_s[i + 1][j];
				}
			}

            _velo_s[V_CAL_MAX_VAL - 1][0] = _V.x;
            _velo_s[V_CAL_MAX_VAL - 1][1] = _V.y;
            _velo_s[V_CAL_MAX_VAL - 1][2] = _V.z;

			subMethod(_velo_s, _A);			//���ٶȼ���
			CurveData::saveAcc(_A);			//��¼���ٶ�����
            //cout << "acc: " << _A << endl;
		}
		_dataCnt++;
	}
}

float Prediction::airResistCalculation(cv::Point3f coordinate, float shootSpeed) {

	float KAPPA = 0.00045f; //����Ŀ�������ϵ��
	float M = 0.04f;        //���������
	float ALPHA = 0.5f;		//����������ֵ����ȡ�Ľ���ϵ����0~1��

	float x = MM_TO_M_TRANS(coordinate.x);
	float z = MM_TO_M_TRANS(coordinate.z);
	float y = -MM_TO_M_TRANS(coordinate.y);
	float speed = shootSpeed;
	float p = sqrt(x * x + z * z);
	float pitchAngleRef = 0.0f;

	//cout << "horizonPosition = " << p << endl;
	//cout << "verticalPosition = " << y << endl;

	float a = -0.5f * GRAVITY * M * M * p * p / (shootSpeed * shootSpeed * (M - KAPPA * ALPHA * p) * (M - KAPPA * ALPHA * p));
	float b = M * p / (M - KAPPA * ALPHA * p);
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
		pitchAngleRef = RADIAN_TO_ANGLE(atanf(y / p));
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

float Prediction::angleCalculation(cv::Point3f coordinate, float shootSpeed) {
	
	//�������ϵ��ת��ʹ��zoxƽ��ˮƽ,y�ᴹֱ 
	float x = MM_TO_M_TRANS(coordinate.x);
	float z = MM_TO_M_TRANS(coordinate.z);
	float y = -MM_TO_M_TRANS(coordinate.y);	//��ֱ���룬������ϵy��������Ϊ���£���ȡ��
	float speed = shootSpeed;
	float tanAngleA = 0.0f;
	float tanAngleB = 0.0f;
	
	float p = sqrtf(x * x + z * z);     //��ȡˮƽ����
	float pitchAngleRef = 0.0f;

	/*
	cout << "horizonPosition = " << p << endl;
	cout << "verticalPosition = " << y << endl;
	*/

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
		pitchAngleRef = RADIAN_TO_ANGLE(atanf(y / p));
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

		//       cout << "tanAngleA = " << tanAngleA << endl;
		//       cout << "tanAngleB = " << tanAngleB << endl;
		//       cout << "pitchAngleRef = " << pitchAngleRef << endl;

	}
	return pitchAngleRef;
}

cv::Point3f Prediction::Iteration(cv::Point3f coordinate_m, float shootSpeed, CarType carType) {
	
	cv::Point3f coordinate_mm;
	coordinate_mm.x = M_TO_MM_TRANS(coordinate_m.x);
	coordinate_mm.y = M_TO_MM_TRANS(coordinate_m.y);
	coordinate_mm.z = M_TO_MM_TRANS(coordinate_m.z);

	float tof, TOF;                                  //�������ʱ�䣬�ɵ����õ�
	float d = sqrtf(coordinate_m.x * coordinate_m.x + coordinate_m.z * coordinate_m.z);		//��Ԥ������µ�ˮƽ����
	cv::Point3f tmpcoord;
	tmpcoord.x = M_TO_MM_TRANS(coordinate_m.x);
	tmpcoord.y = M_TO_MM_TRANS(coordinate_m.y);
	tmpcoord.z = M_TO_MM_TRANS(coordinate_m.z);
	float beta = angleCalculation(tmpcoord, shootSpeed);   //pitch��Ƕ�
	//cout << "beta: " << beta << endl;
	float v = cosf(ANGLE_TO_RADIAN(beta)) * shootSpeed;    //�ٶȵ�ˮƽ����
	//cout << "v: " << v << endl;
	TOF = d / v;   //�õ���Ԥ������µķ���ʱ��
	//cout << "shootSpeed: " << shootSpeed;
	//cout << "distance d: " << d << endl;
	//cout << "no pre TOF" << TOF << endl;

	//��λΪm
	float x = 0.0f, y = 0.0f, z = 0.0f;
	float temp = 1;
	int cnt = 0;
	//��ʼ����
    //��ʱ���С��10��5�η��ҵ�������С��15
	//��һ�ε���TΪ��Ԥ�з���ʱ��
    while (fabs(temp) > 1e-5 && cnt < 15) {
        x = coordinate_m.x + _V.x * TOF + _A.x * TOF * TOF / 2.0f;
        //y = coordinate_m.y + _V.y * TOF + _A.y * TOF * TOF / 2.0f;
        z = coordinate_m.z + _V.z * TOF + _A.z * TOF * TOF / 2.0f;

        //x = coordinate_m.x + _V.x * TOF;
        //y = coordinate_mm.y + _V.y * TOF;
        y = coordinate_m.y;
        //z = coordinate_m.z + _V.z * TOF;

		//Ԥ��λ�õ�ˮƽ����
		float D = sqrtf(x * x + z * z);
		//cout << "D: " << D << endl;
		//�õ�Ԥ�����ʱ��
		cv::Point3f coord;
		coord.x = M_TO_MM_TRANS(x);
		coord.y = M_TO_MM_TRANS(y);
		coord.z = M_TO_MM_TRANS(z);
		//��������ĵ�λΪm
		if (carType == OLD_HERO || carType == NEW_HERO) {
			beta = airResistCalculation(coordinate_mm, shootSpeed);
		}
		else {
			beta = angleCalculation(coordinate_mm, shootSpeed);     //��õ�����pitch��Ƕ�
		}

        //cout << "beta: " << beta << endl;
		v = shootSpeed * cosf(ANGLE_TO_RADIAN(beta));
		if (v > 0) {
			tof = D / v;  //Ԥ��ķ���ʱ��
		}
		else {
			tof = TOF;
		}
		//���ʱ���
		temp = TOF - tof;
		//�������� Ԥ�е�ľ��������һ�ι۲��Ĳ�ֵ
		float delta = fabs(temp);
        /*
		cout << "TOF: " << TOF << endl;
		cout << "tof: " << tof << endl;
		cout << "temp: " << temp << endl;
		cout << endl;
        */
		//������������
		if (temp > 0) {
			TOF -= delta;
		}
		else {
			TOF += delta;
		}
		cnt++;
	}

    if (cnt >= 15) {
		cout << "prediction interation fail!" << endl;
	}

	/*
	cout << " origin x: " << coordinate_m.x << endl;
	cout << " origin y: " << coordinate_m.y << endl;
	cout << " origin z: " << coordinate_m.z << endl;
	cout << "cnt: " << cnt << endl;
	cout << "final TOF: " << tof << endl;
	cout << " x: " << M_TO_MM_TRANS(x);
	cout << " y: " << M_TO_MM_TRANS(y);
	cout << " z: " << M_TO_MM_TRANS(z) << endl << endl;
	*/

	return cv::Point3f(x, y, z);
}

cv::Point3f Prediction::prediction(cv::Point3f coordinate, float time, float shootSpeed, CarType carType) {

	getDeltaTime(time);
    dataCal(coordinate);

	//��ȡ�������ݣ���ת��Ϊm
	float x_m, y_m, z_m;
	x_m = MM_TO_M_TRANS(coordinate.x);
	y_m = MM_TO_M_TRANS(coordinate.y);
	z_m = MM_TO_M_TRANS(coordinate.z);

	//ʱ��������
	if (_deltaTime_ms > MAX_TIME_BIAS) {
		//LOG::debug("Prediction _deltaTime_s is " + std::to_string(_deltaTime_s) + " ms! timed out! ");
		return coordinate;
	}

	float fabVx, fabVy, fabVz;
	fabVx = fabs(_V.x);
	fabVy = fabs(_V.y);
	fabVz = fabs(_V.z);

	//�������Ԥ���ٶ��޷�
	if (fabVx > PREDICT_MAX_SPEED) {
		//LOG::debug(" Predict speed Limit! vx: " + std::to_string(coordinateV.x));
		(_V.x > 0) ? _V.x = PREDICT_MAX_SPEED : _V.x = -PREDICT_MAX_SPEED;
	}
	if (fabVy > PREDICT_MAX_SPEED) {
		//LOG::debug(" Predict speed Limit! vy: " + std::to_string(coordinateV.y));
		(_V.y > 0) ? _V.y = PREDICT_MAX_SPEED : _V.y = -PREDICT_MAX_SPEED;
	}
	if (fabVz > PREDICT_MAX_SPEED) {
		//LOG::debug(" Predict speed Limit! vz: " + std::to_string(coordinateV.z));
		(_V.z > 0) ? _V.z = PREDICT_MAX_SPEED : _V.z = -PREDICT_MAX_SPEED;
	}

	//��������
	if (fabVx < PREDICT_MIN_SPEED) {
		_V.x = 0.0f;
	}
	if (fabVy < PREDICT_MIN_SPEED) {
		_V.y = 0.0f;
	}
	if (fabVz < PREDICT_MIN_SPEED) {
		_V.z = 0.0f;
	}

	float fabAx, fabAy, fabAz;
	fabAx = fabs(_A.x);
	fabAy = fabs(_A.y);
	fabAz = fabs(_A.z);

	//�ͼ��ٶ�����
	if (fabAx < PREDICT_MIN_ACC) {
		_A.x = 0.0f;
	}
	if (fabAy < PREDICT_MIN_ACC) {
		_A.y = 0.0f;
	}
	if (fabAz < PREDICT_MIN_ACC) {
		_A.z = 0.0f;
	}

	//�������Ԥ�м��ٶ��޷�
	if (fabAx > PREDICT_MAX_ACC) {
		//	LOG::debug(" Predict acc Limit! ax: " + std::to_string(coordinateA.x));
		(_A.x > 0) ? _A.x = PREDICT_MAX_ACC : _A.x = -PREDICT_MAX_ACC;
	}
	if (fabAy > PREDICT_MAX_ACC) {
		//	LOG::debug(" Predict acc Limit! ay: " + std::to_string(coordinateA.y));
		(_A.y > 0) ? _A.y = PREDICT_MAX_ACC : _A.y = -PREDICT_MAX_ACC;
	}
	if (fabAz > PREDICT_MAX_ACC) {
		//	LOG::debug(" Predict acc Limit! az: " + std::to_string(coordinateA.z));
		(_A.z > 0) ? _A.z = PREDICT_MAX_ACC : _A.z = -PREDICT_MAX_ACC;
	}

	//��ʱ����
	float x_m_noBias = x_m;
	float z_m_noBias = z_m;

    //x_m += _V.x * _timeBias;
	//y_m += _V.y * _timeBias;
    //z_m += _V.z * _timeBias;
    x_m += _V.x * _timeBias + _A.x * _timeBias * _timeBias / 2.0f;
	//y_m += _V.y * _timeBias + _A.y * _timeBias * _timeBias / 2.0f;
    z_m += _V.z * _timeBias + _A.z * _timeBias * _timeBias / 2.0f;

	//������ⵯ�����
	cv::Point3f coordinateResult;
	cv::Point3f coordinateNoBias;
	coordinateResult = Iteration(cv::Point3f(x_m, y_m, z_m), shootSpeed, carType);

	if( CurveData::isEnable() )	coordinateNoBias = Iteration(cv::Point3f(x_m_noBias, y_m, z_m_noBias), shootSpeed, carType);

    /*
    cout << " vx: " << _V.x << " vy: " << _V.y << " vz: " << _V.z << endl;
    cout << " ax: " << _A.x << " ay: " << _A.y << " az: " << _A.z << endl;
    */

    cv::Point3f finalCoordinate = coordinateResult;
	if (isnan(coordinateResult.x) || isnan(coordinateResult.y) || isnan(coordinateResult.z)) {
        //LOG::debug(" Predict coordinate is nan! ");
		return coordinate;
	}
	else {

        //finalCoordinate = _predictionFilter.KPredictionRun(coordinateResult);
		if (CurveData::isEnable()) {

			coordinateNoBias.x = M_TO_MM_TRANS(coordinateNoBias.x);
			coordinateNoBias.y = M_TO_MM_TRANS(coordinateNoBias.y);
			coordinateNoBias.z = M_TO_MM_TRANS(coordinateNoBias.z);

			CurveData::saveNoBiasCoordinate(coordinateNoBias);
		}

		//�������굥λΪmm
		finalCoordinate.x = M_TO_MM_TRANS(coordinateResult.x);
		finalCoordinate.y = M_TO_MM_TRANS(coordinateResult.y);
		finalCoordinate.z = M_TO_MM_TRANS(coordinateResult.z);
		return finalCoordinate;
	}
}

void CurveData::saveTime(float time) {
	if (instance().CURVE_SWITCH == true) {
		instance()._nowTime_ms.push_back(time);
		instance()._isEmpty = false;
	}
}

bool CurveData::isEnable() {
	return instance().CURVE_SWITCH;
}

void CurveData::saveVelocity(cv::Point3f origin, cv::Point3f filter) {
	if (instance().CURVE_SWITCH == true) {
		instance()._velocityOrigin.push_back(origin);
		instance()._velocityFilter.push_back(filter);
		instance()._isEmpty = false;
	}
}

void CurveData::saveAcc(cv::Point3f origin) {
	if (instance().CURVE_SWITCH == true) {
		instance().accOrigin.push_back(origin);
		instance()._isEmpty = false;
	}
}

void CurveData::saveNoBiasCoordinate(cv::Point3f coordinate) {
	if (instance().CURVE_SWITCH == true) {
		instance()._coordinateNoBias = coordinate;
		instance()._isEmpty = false;
	}
}

cv::Point3f CurveData::getNoBiasCoordinate() {
	return instance()._coordinateNoBias;
}

void CurveData::saveCoordinate(cv::Point3f origin, cv::Point3f filter, cv::Point3f prediction) {
	if (instance().CURVE_SWITCH == true) {
		instance()._coordinateOrigin.push_back(origin);
		instance()._coordinateFilter.push_back(filter);
		instance()._coordinatePredict.push_back(prediction);
		instance()._isEmpty = false;
	}
}

void CurveData::saveAngle(cv::Point2f angleNoBias, cv::Point2f angleRef, cv::Point2f angleFbd) {
    if (instance().CURVE_SWITCH == true) {
        instance()._angleTime.push_back(cv::getTickCount() / cv::getTickFrequency() * 1000);
		instance()._angleNoBias.push_back(angleNoBias);
        instance()._angleRef.push_back(angleRef);
        instance()._angleFbd.push_back(angleFbd);
        instance()._isEmpty = false;
    }
}

void CurveData::clear() {
	instance()._nowTime_ms.clear();
	instance()._coordinateOrigin.clear();
	instance()._coordinatePredict.clear();
	instance()._coordinateFilter.clear();
	instance()._velocityOrigin.clear();
    instance()._velocityFilter.clear();
	instance().accOrigin.clear();
    instance()._angleNoBias.clear();
    instance()._angleRef.clear();
    instance()._angleFbd.clear();
    instance()._angleTime.clear();
	instance()._isEmpty = true;
};

void CurveData::write() {

	if (instance()._writeCnt > instance().MAX_WRITE_CNT && instance().CURVE_SWITCH == true) {

		ofstream outfile;
		string str1 = "coordinateOrigin";
		outfile.open(CURVE_DATA_PATH + str1);
		for (int i = 0; i < instance()._coordinateOrigin.size(); i++) {
			outfile << instance()._coordinateOrigin[i].x;
			if (i != instance()._coordinateOrigin.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
		for (int i = 0; i < instance()._coordinateOrigin.size(); i++) {
			outfile << instance()._coordinateOrigin[i].y;
			if (i != instance()._coordinateOrigin.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
		for (int i = 0; i < instance()._coordinateOrigin.size(); i++) {
			outfile << instance()._coordinateOrigin[i].z;
			if (i != instance()._coordinateOrigin.size() - 1)
				outfile << ", ";
		}
		outfile.close();
        cout << "count: " << instance()._coordinateOrigin.size() << endl;
		cout << "write file: " << str1 << "finish !" << endl;



		string str2 = "coordinatePredict";
		outfile.open(CURVE_DATA_PATH + str2);
		for (int i = 0; i < instance()._coordinatePredict.size(); i++) {
			outfile << instance()._coordinatePredict[i].x;
			if (i != instance()._coordinatePredict.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
		for (int i = 0; i < instance()._coordinatePredict.size(); i++) {
			outfile << instance()._coordinatePredict[i].y;
			if (i != instance()._coordinatePredict.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
		for (int i = 0; i < instance()._coordinatePredict.size(); i++) {
			outfile << instance()._coordinatePredict[i].z;
			if (i != instance()._coordinatePredict.size() - 1)
				outfile << ", ";
		}
		outfile.close();
        cout << "count: " << instance()._coordinatePredict.size() << endl;
		cout << "write file: " << str2 << "finish !" << endl;



		string str3 = "coordinateFilter";
		outfile.open(CURVE_DATA_PATH + str3);
		for (int i = 0; i < instance()._coordinateFilter.size(); i++) {
			outfile << instance()._coordinateFilter[i].x;
			if (i != instance()._coordinateFilter.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
		for (int i = 0; i < instance()._coordinateFilter.size(); i++) {
			outfile << instance()._coordinateFilter[i].y;
			if (i != instance()._coordinateFilter.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
		for (int i = 0; i < instance()._coordinateFilter.size(); i++) {
			outfile << instance()._coordinateFilter[i].z;
			if (i != instance()._coordinateFilter.size() - 1)
				outfile << ", ";
		}
		outfile.close();
        cout << "count: " << instance()._coordinateFilter.size() << endl;
		cout << "write file: " << str3 << "finish !" << endl;



		string str4 = "coordinateTime";
		outfile.open(CURVE_DATA_PATH + str4);
		for (int i = 0; i < instance()._nowTime_ms.size(); i++) {
			outfile << instance()._nowTime_ms[i];
			if (i != instance()._nowTime_ms.size() - 1)
				outfile << ", ";
		}
		outfile.close();
		cout << "write file: " << str4 << "finish !" << endl;



        string str6 = "velocityOrigin";
		outfile.open(CURVE_DATA_PATH + str6);
        for (int i = 0; i < instance()._velocityOrigin.size(); i++) {
            outfile << instance()._velocityOrigin[i].x;
            if (i != instance()._velocityOrigin.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
        for (int i = 0; i < instance()._velocityOrigin.size(); i++) {
			outfile << instance()._velocityFilter[i].y;
            if (i != instance()._velocityOrigin.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
        for (int i = 0; i < instance()._velocityOrigin.size(); i++) {
            outfile << instance()._velocityOrigin[i].z;
            if (i != instance()._velocityOrigin.size() - 1)
				outfile << ", ";
		}
		outfile.close();
		cout << "write file: " << str6 << "finish !" << endl;



        string str7 = "velocityFilter";
        outfile.open(CURVE_DATA_PATH + str7);
        for (int i = 0; i < instance()._velocityFilter.size(); i++) {
            outfile << instance()._velocityFilter[i].x;
            if (i != instance()._velocityFilter.size() - 1)
                outfile << ", ";
        }
        outfile << "\n";
        for (int i = 0; i < instance()._velocityFilter.size(); i++) {
            outfile << instance()._velocityFilter[i].y;
            if (i != instance()._velocityFilter.size() - 1)
                outfile << ", ";
        }
        outfile << "\n";
        for (int i = 0; i < instance()._velocityFilter.size(); i++) {
            outfile << instance()._velocityFilter[i].z;
            if (i != instance()._velocityFilter.size() - 1)
                outfile << ", ";
        }
        outfile.close();
        cout << "write file: " << str7 << "finish !" << endl;

		

        string str8 = "angleData";
        outfile.open(CURVE_DATA_PATH + str8);

		for (int i = 0; i < instance()._angleNoBias.size(); i++) {
			outfile << instance()._angleNoBias[i].x;
			if (i != instance()._angleNoBias.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
		for (int i = 0; i < instance()._angleNoBias.size(); i++) {
			outfile << instance()._angleNoBias[i].y;
			if (i != instance()._angleNoBias.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
        for (int i = 0; i < instance()._angleRef.size(); i++) {
            outfile << instance()._angleRef[i].x;
            if (i != instance()._angleRef.size() - 1)
                outfile << ", ";
        }
        outfile << "\n";
        for (int i = 0; i < instance()._angleRef.size(); i++) {
            outfile << instance()._angleRef[i].y;
            if (i != instance()._angleRef.size() - 1)
                outfile << ", ";
        }
        outfile << "\n";
        for (int i = 0; i < instance()._angleFbd.size(); i++) {
            outfile << instance()._angleFbd[i].x;
            if (i != instance()._angleFbd.size() - 1)
                outfile << ", ";
        }
        outfile << "\n";
        for (int i = 0; i < instance()._angleFbd.size(); i++) {
            outfile << instance()._angleFbd[i].y;
            if (i != instance()._angleFbd.size() - 1)
                outfile << ", ";
        }
        outfile << "\n";
        for (int i = 0; i < instance()._angleTime.size(); i++) {
            outfile << instance()._angleTime[i];
            if (i != instance()._angleTime.size() - 1)
                outfile << ", ";
        }
        outfile.close();
        cout << "write file: " << str8 << "finish !" << endl;

		

		string str9 = "accOrigin";
        outfile.open(CURVE_DATA_PATH + str9);
		for (int i = 0; i < instance().accOrigin.size(); i++) {
			outfile << instance().accOrigin[i].x;
			if (i != instance().accOrigin.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
		for (int i = 0; i < instance().accOrigin.size(); i++) {
			outfile << instance().accOrigin[i].y;
			if (i != instance().accOrigin.size() - 1)
				outfile << ", ";
		}
		outfile << "\n";
		for (int i = 0; i < instance().accOrigin.size(); i++) {
			outfile << instance().accOrigin[i].z;
			if (i != instance().accOrigin.size() - 1)
				outfile << ", ";
		}
		outfile.close();
		cout << "write file: " << str9 << "finish !" << endl;



		clear();
		instance()._writeCnt = 0;
	}
	instance()._writeCnt++;
};

bool CurveData::isEmpty() {
	return instance()._isEmpty;
}

