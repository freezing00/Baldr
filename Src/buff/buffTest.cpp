#include"buffTest.h"
#include<cmath>
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#define BUFFTESTDEBUG 1
//#define BUFFTESTSAVER 1 //保存样本
/*  调试窗口如下
	cv::namedWindow("BuffTestWindow", 0);
	cv::namedWindow("ArmorROIWindow", 0);
	cv::namedWindow("RMat", 0);
*/

//----------------------_SaveImage----------------------------
//样本保存器定义
//构造函数
_Buff_SaveImage::_Buff_SaveImage(const char* adress) {
	_saveAdress = new std::string(adress);
	_now = 0;
}


//析构函数
_Buff_SaveImage::~_Buff_SaveImage() {
	delete _saveAdress;
}


//保存
void _Buff_SaveImage::save(cv::Mat& img) {
	if (img.empty())return;
	//将now变为字符串
	std::string filename = std::to_string(_now++);
	filename += ".png";
	cv::imwrite(*_saveAdress + filename, img);
}



//------------------------Buff--------------------------------
Buff::Buff() {
	_Recognizer = cv::ml::ANN_MLP::load("../trainData/test.xml");
	_saveSample = new _Buff_SaveImage("../save/");
}


void Buff::setMat(cv::Mat& img, bool ownColor) {
	if (img.empty())return;
	_init(img, ownColor);
	_MAKEROI();//作ROI
	_preprocess();//预处理
	_findContoursAndRects();//找轮廓
	_findRFlag = _findR();//寻找R
	_findArmors();//寻找装甲板
	_findArmorsROIs();//迭代最优装甲板
#ifdef  BUFFTESTDEBUG
//	cv::imshow("BuffTestWindow", _DEBUGMAT);
//	cv::waitKey(1);
#endif //  BUFFTESTDEBUG
	_UPDATEROI();
}


void Buff::_init(cv::Mat& img, bool ownColor) {
	img.copyTo(_originMat);//复制原图像
	_ownColor = ownColor;//存储我方颜色
	_findRFlag = false;//初始化没有找到R
	_targetArmorIndex = -1;//初始化没有找到目标装甲板
}


//进行图像预处理
void Buff::_preprocess() {
	cv::cvtColor(_ROI, _grayMat, cv::COLOR_BGR2GRAY);
	cv::threshold(_grayMat, _separationSrcWhite, 200, 255, cv::THRESH_BINARY);
	cv::bitwise_not(_separationSrcWhite, _separationSrcWhite);
	_splitedMats.clear();
	cv::split(_ROI, _splitedMats);
	cv::Mat temp_1, temp_2;
	int index_1, index_2, index_3;
	if (_ownColor == 1) {//我方为蓝
		index_1 = 2; index_2 = 1; index_3 = 0;
	}
	else {//我方为红色
		index_1 = 0; index_2 = 1; index_3 = 2;
	}
	cv::subtract(_splitedMats[index_1], _splitedMats[index_2], temp_1);
	cv::subtract(_splitedMats[index_1], _splitedMats[index_3], temp_2);
	cv::threshold(_grayMat, _binaryMat, _newBuffPara.grayThresholdValue, 255, cv::THRESH_BINARY);
	cv::threshold(temp_1, temp_1, _newBuffPara.channelThresholdValue, 255, cv::THRESH_BINARY);
	cv::threshold(temp_2, temp_2, _newBuffPara.channelThresholdValue, 255, cv::THRESH_BINARY);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Point p = cv::Point(-1, -1);
	cv::dilate(temp_1, temp_1, kernel, p, _newBuffPara.singleIterations);
	cv::dilate(temp_2, temp_2, kernel, p, _newBuffPara.singleIterations);
    _binaryMat = _binaryMat & temp_1 & temp_2 & _separationSrcWhite&_separationSrcWhite;
	//膨胀
	cv::dilate(_binaryMat, _binaryMat, kernel, p, _newBuffPara.binMatIterations);
	//开闭操作
	//cv::morphologyEx(_binaryMat, _binaryMat, cv::MORPH_CLOSE, kernel);
	//cv::morphologyEx(_binaryMat, _binaryMat, cv::CV_MOP_OPEN,  kernel);
}


//获得所有轮廓与其旋转矩形
void Buff::_findContoursAndRects() {
	_hierachy.clear();//清空拓扑次序数组
	_contours.clear();//清空轮廓数组
	_Rects.clear();
	//查找轮廓信息
	cv::findContours(_binaryMat, _contours, _hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
	if (_contours.size() == 0)return;
	//根据轮廓面积过滤一波轮廓
	std::vector<std::vector<cv::Point>>::iterator contour = _contours.begin();
	std::vector<cv::Vec4i>::iterator hierachy = _hierachy.begin();
	std::vector<cv::RotatedRect>::iterator rect;
	while (contour != _contours.end() && hierachy != _hierachy.end()) {
		double area = cv::contourArea(*contour);
		if (area > 518400 || area < 100) {
			contour = _contours.erase(contour);
			hierachy = _hierachy.erase(hierachy);
		}
		else {
			contour++;
			hierachy++;
		}
	}
	//获得旋转矩形
	for (long unsigned int i = 0; i < _contours.size(); i++) {
		_Rects.push_back(minAreaRect(_contours[i]));
	}
	//对旋转矩形进行一波过滤
	contour = _contours.begin();
	hierachy = _hierachy.begin();
	rect = _Rects.begin();
	while (contour != _contours.end() && hierachy != _hierachy.end() && rect != _Rects.end()) {
		//外包限制矩形
		bool limit_rect = rect->size.width < 5.f || rect->size.height < 5.f || rect->size.width>250 || rect->size.height>250;
		if (cv::contourArea(*contour) < 25.0f || limit_rect) {
			//删除这个节点
			contour = _contours.erase(contour);
			hierachy = _hierachy.erase(hierachy);
			rect = _Rects.erase(rect);
		}
		else {
			contour++;
			hierachy++;
			rect++;
		}
	}
}


bool Buff::_findR() {
	//迭代最优解
	int bestindex = -1;
	float bestscore = -9999;
	std::vector<std::vector<cv::Point>>::iterator contour = _contours.begin();
	std::vector<cv::RotatedRect>::iterator rect = _Rects.begin();
	int i = 0;
	//遍历所有轮廓
	while (rect != _Rects.end() && contour != _contours.end()) {
//       if(_hierachy[i][3]!=-1||_hierachy[i][2]!=-1){//存在子轮廓或者父轮廓就不处理
//           rect++;
//           contour++;
//           i++;
//           continue;
//        }
		if (_RMorphologicalDiscrimination(*rect)) {
            //float tempscore = _RAIDiscrimination(*rect);
            //if (tempscore > bestscore) {
				bestindex = i;
                //bestscore = tempscore;
            //}
		}
		rect++;
		contour++;
		i++;
	}
	if (bestindex != -1) {
		_RresultRect = _Rects[bestindex].boundingRect();
#ifdef BUFFTESTDEBUG
		_drawRotatedRect(_DEBUGMAT, _Rects[bestindex], 0, 0, 255);
#endif
		return true;
	}
	return false;
}


//R形态辨别
bool Buff::_RMorphologicalDiscrimination(cv::RotatedRect& rect) {
	double longSide = _getRotatedRectSide(rect, true);//获得长边
	double shortSide = _getRotatedRectSide(rect, false);//获得短边
	if (longSide < shortSide * _r_Para.squreSideRatio) {
        //cout<<"long"<<longSide<<endl;
        //cout<<"short"<<shortSide<<endl;
		if (longSide > _r_Para.squreSizeLimit_MIN && shortSide > _r_Para.squreSizeLimit_MIN && longSide < _r_Para.squreSizeLimit_MAX && shortSide < _r_Para.squreSizeLimit_MAX) {
			return true;
		}
	}
	return false;
}


//R机器学习辨别
float Buff::_RAIDiscrimination(cv::RotatedRect& rect) {
	//获得外包
	cv::Rect ROIRect = rect.boundingRect();
	cv::Mat R = _makeRImage(rect);
	cv::Mat result;
	if (R.empty())return -9999;
	if (!R.empty()) {
		cv::resize(R, R, cv::Size(30, 30), 0, 0);
		cv::Mat input(1, 30 * 30, CV_32FC1);
		int now = 0;
		for (int j = 0; j < 30; j++)
		{
			for (int k = 0; k < 30; k++) {
				input.at<float>(0, now++) = (float)R.at<uchar>(j, k) / 255.0;
			}
		}
		_Recognizer->predict(input, result);
#ifdef BUFFTESTDEBUG
		//std::cout << "-------------------" << result.at<float>(0, 0) << std::endl;
        //cv::imshow("RMat",R);
#ifdef BUFFTESTSAVER
		_saveSample->save(R);
#endif
#endif // BUFFTESTDEBUG
		return result.at<float>(0, 0);//返回置信度
	}
	return -9999;
}


//获得旋转矩形的长边与短边 true长 false短
float Buff::_getRotatedRectSide(cv::RotatedRect& rect, bool flag) {
	if (flag) {//短边
		return rect.size.width > rect.size.height ? rect.size.width : rect.size.height;
	}
	else {//长边
		return rect.size.width < rect.size.height ? rect.size.width : rect.size.height;
	}
}


//截取中心R
cv::Mat Buff::_makeRImage(cv::RotatedRect& rect) {
	cv::Rect2f boundingRect = rect.boundingRect();
	_resizeRect(boundingRect, 1.5);
	cv::Rect ROI = cv::Rect(boundingRect.tl().x, boundingRect.tl().y, boundingRect.width, boundingRect.height);
	if (_testROI(ROI)) {
		return _grayMat(ROI);
	}
	else {
		cv::Mat Blank;
		return Blank;
	}
}


//根据R旋转矩形稍微扩大旋转矩形，为截取ROI服务
void Buff::_resizeRect(cv::Rect2f& rect, float u) {
	if (u <= 1.0f) {
		return;
	}
	rect.x -= float(rect.width) * (u - 1.0f) * 0.5f;
	rect.y -= float(rect.height) * (u - 1.0f) * 0.5f;
	rect.width += float(rect.width) * (u - 1.0f);
	rect.height += float(rect.height) * (u - 1.0f);
	if (rect.tl().y < 1.0f) {
		rect += cv::Point2f(0.0f, fabs(rect.tl().y) + 1);
	}
	if (rect.br().y > _ROI.rows - 1) {
		rect -= cv::Point2f(0.0f, rect.br().y + 1 - _ROI.rows);
	}
	if (rect.tl().x < 1.0f) {
		rect += cv::Point2f(fabs(rect.tl().x) + 1, 0.0f);
	}
	if (rect.br().x > _ROI.cols - 1) {
		rect -= cv::Point2f(rect.br().x + 1 - _ROI.cols, 0.0f);
	}
}


//检测ROI范围是否合法
bool Buff::_testROI(cv::Rect& rect) {
	if (rect.tl().x > 1.0 && rect.br().x < _ROI.cols - 1 && rect.tl().y > 1 && rect.br().y < _ROI.rows - 1 && rect.tl().x < rect.br().x - 5 && rect.tl().y < rect.br().y - 5) {
		return true;
	}
	else {
		return false;
	}
}
bool Buff::_testROI(cv::Rect2f& rect) {
	if (rect.tl().x > 1.0 && rect.br().x < _ROI.cols - 1 && rect.tl().y > 1 && rect.br().y < _ROI.rows - 1 && rect.tl().x < rect.br().x - 5 && rect.tl().y < rect.br().y - 5) {
		return true;
	}
	else {
		return false;
	}
}


//寻找装甲板
void Buff::_findArmors() {
	_ArmorIndex.clear();
	std::vector<cv::RotatedRect>::iterator rect = _Rects.begin();
	std::vector<cv::Vec4i>::iterator hierachy = _hierachy.begin();
	//遍历所有轮廓
	int i = 0;
	while (rect != _Rects.end() && hierachy != _hierachy.end()) {
		if ((*hierachy)[2] == -1 && _armorsShapeDetection(*rect)) {
			_ArmorIndex.push_back(i);
		}
		rect++;
		hierachy++;
		i++;
	}
}


//装甲板形态辨别
bool Buff::_armorsShapeDetection(cv::RotatedRect& rect) {
	int rect_long = _getRotatedRectSide(rect, true);
	int rect_short = _getRotatedRectSide(rect, false);
	//找到了中心R
	if (_findRFlag) {
        //cout<<"rate"<<(double)rect_long / (double)rect_short<<endl;
		//装甲板长短边比例判断
        if (!((double)rect_long / (double)rect_short > _newBuffPara.armorSideRatio_MIN && (double)rect_long / (double)rect_short<_newBuffPara.armorSideRatio_MAX)) {
            return false;
        }
		//限制装甲板面积:根据找到的R的大小
		float rectArea = (float)rect.size.width * (float)rect.size.height;
        //cout<<"rectarea"<<rectArea/_RresultRect.area()<<endl;
		if (!(rectArea > _newBuffPara.armorTo_R_SizeRatio_MIN * _RresultRect.area() && rectArea < _newBuffPara.armorTo_R_SizeRatio_MAX * _RresultRect.area())) {
            return false;

		}
		//限制装甲板距离R的大小:根据R矩形的边的长短
		double distance = _calculatePointsDistance(_getRectCenter(_RresultRect), rect.center);
        //cout<<"distance"<<distance/_RresultRect.size().width <<endl;
		if (!(distance > _RresultRect.size().width * _newBuffPara.armorTo_R_DistanceRatio_MIN && distance < _RresultRect.size().width * _newBuffPara.armorTo_R_DistanceRatio_MAX)) {
            return false;

		}
		//装甲板中心与到R直线夹角限制[50,90]限制
		if (_limitArmorAndRAngle(rect) == false) { return false; };
		//这个矩形符合形态上的装甲板
		return true;
	}
	else {//没有找到中心R
		return false;
	}
}

bool Buff::_limitArmorAndRAngle(cv::RotatedRect& rect) {
	//得到两个直线方程
	static float A[2], B[2];
	//计算Armor-R直线
	cv::Point2f RCenter = _getRectCenter(_RresultRect);
	A[0] = rect.center.y - RCenter.y;
	B[0] = RCenter.x - rect.center.x;
	//计算Armor上面为长边的位置的直线
	static bool shortIs_0_1;
	static cv::Point2f pts[4];
	rect.points(pts);
	if (_calculatePointsDistance(pts[0], pts[1]) < _calculatePointsDistance(pts[0], pts[3])) {
		shortIs_0_1 = true;
	}
	else {
		shortIs_0_1 = false;
	}
	//计算两个直线的夹角
	static cv::Point2f ArmorP1, ArmorP2;
	ArmorP1.x = pts[0].x; ArmorP1.y = pts[0].y;
	if (shortIs_0_1 == false) {//0-1为长边
		ArmorP2.x = pts[1].x; ArmorP2.y = pts[1].y;
	}
	else {//0-3为长边
		ArmorP2.x = pts[3].x; ArmorP2.y = pts[3].y;
	}
	//计算A[1] B[1]
	A[1] = ArmorP1.y - ArmorP2.y;
	B[1] = ArmorP2.x - ArmorP1.x;
	if (_calculateLineAndLineAngle(A[0], B[0], A[1], B[1])<65.0f) {//夹角小于50则不是装甲板
		return false;
	}
	//返回:是否符合要求
	return true;
}

//获得矩形的中心点
cv::Point Buff::_getRectCenter(cv::Rect& rect) {
	cv::Point center = rect.tl();
	center.x += rect.width / 2.0;
	center.y += rect.height / 2;
	return center;
}


//计算两点间的距离
double Buff::_calculatePointsDistance(cv::Point2f p1, cv::Point2f p2) {
	double dx = p1.x < p2.x ? p2.x - p1.x : p1.x - p2.x;
	double dy = p1.y < p2.y ? p2.y - p1.y : p1.y - p2.y;
	return sqrt(dx * dx + dy * dy);
}

//计算直线与直线的夹角(0,90]
//方程为一般式直线方程 0=Ax+Bx+C
float Buff::_calculateLineAndLineAngle(float A_1, float B_1, float A_2, float B_2) {
	float k1, k2;
	k1 = -A_1 / B_1;
	k2 = -A_2 / B_2;
	float tan_a = fabs((k2 - k1) / (1.0f + k2 * k1));
	//atan其返回值为[-pi / 2, +pi / 2]之间的一个数
	//返回结果为度数
	return (atan(tan_a)/(BUFF_PI/2.0f))*90.0f;
}

//获得装甲板悬臂ROI
void Buff::_findArmorsROIs() {
	//迭代取最优
	float score = 99999;
	_targetArmorIndex = -1;//初始化为-1代表没有找到目标
	for (unsigned long int i = 0; i < _ArmorIndex.size(); i++) {
		int armor_width = _Rects[_ArmorIndex[i]].size.width;
		int armor_height = _Rects[_ArmorIndex[i]].size.height;
		int height = (float)armor_height * _newBuffPara.shortSideRatioForROI;
		int width = (float)armor_width * _newBuffPara.longSideRatioForROI;
		if (_Rects[_ArmorIndex[i]].size.height > _Rects[_ArmorIndex[i]].size.width) {
			height = (float)armor_height * _newBuffPara.longSideRatioForROI;
			width = (float)armor_width * _newBuffPara.shortSideRatioForROI;
		}
		cv::Point ArmorROICenter = _getArmorROICenter(_getRectCenter(_RresultRect), _Rects[_ArmorIndex[i]].center, 0.75);
		cv::RotatedRect ROIRect(ArmorROICenter, cv::Size(width, height), _Rects[_ArmorIndex[i]].angle);
		//ROI图像
		cv::Mat ROIMat;
		ROIMat = _ArmorROIProcess(ROIRect);
		if (ROIMat.empty())continue;
		float tempScore = _dealFeature(ROIMat);
		if (tempScore < score) {
			score = tempScore;
			_targetArmorIndex = _ArmorIndex[i];//更新最优解
		}
#ifdef BUFFTESTDEBUG
		_drawRotatedRect(_DEBUGMAT, ROIRect, 255, 0, 0);
#endif
	}
#ifdef BUFFTESTDEBUG
	//画出目标
	if (_targetArmorIndex >= 0)
		_drawRotatedRect(_DEBUGMAT, _Rects[_targetArmorIndex], 0, 255, 0);
#endif
}


//计算装甲板悬臂ROI中心点
cv::Point Buff::_getArmorROICenter(cv::Point RCenter, cv::Point ArmorCenter, float u) {
	cv::Point2f temp_vector;
	temp_vector.x = ArmorCenter.x - RCenter.x;
	temp_vector.y = ArmorCenter.y - RCenter.y;
	temp_vector.x *= u;
	temp_vector.y *= u;
	temp_vector.x += RCenter.x;
	temp_vector.y += RCenter.y;
	return temp_vector;
}


cv::Mat Buff::_ArmorROIProcess(cv::RotatedRect rect) {
	cv::Rect2f bounding = rect.boundingRect();
	_resizeRect(bounding, 1.1);
	if (_testROI(bounding) == false) {
		cv::Mat null;
		return null;
	}
	cv::Mat ROI = _binaryMat(bounding).clone();
	std::vector<cv::Point>ROIPoints;
	cv::Point2f pts[4];
	rect.points(pts);

//ROI抠图
	/*
	方程序号  经过点
	0		   0-1
	1          1-2
	2          2-3
	3          3-0
	*/
	//计算四个线性方程 0=Ax+By+C 坐标系为_ROI为基准
	//A = Y2 - Y1	B = X1 - X2		C = X2 * Y1 - X1 * Y2
	static float A[4], B[4], C[4], Flag[4];
	float testX[4] = { 0 ,0 ,_ROI.cols ,_ROI.cols };
	float testY[4] = { _ROI.rows ,0,0,_ROI.rows };
	for (int i = 0; i < 4; i++) {
		A[i] = pts[(i + 1) % 4].y - pts[i].y;
		B[i] = pts[i].x - pts[(i + 1) % 4].x;
		C[i] = pts[(i + 1) % 4].x * pts[i].y - pts[i].x * pts[(i + 1) % 4].y;
		if (A[i] * testX[i] + B[i] * testY[i] + C[i] < 0.0f) {
			Flag[i] = 1;
		}
		else {
			Flag[i] = -1;
		}
	}
//去掉中间悬臂
	//方案:以涂黑旋转中心为圆心，半径为19/87 rect.longside内的区域
	float radiusLimit = (19.0 / 87.0) * _getRotatedRectSide(rect, true);
//遍历ROI像素点
	cv::Point2f nowPointer;
	for (size_t x = 0; x < ROI.cols; ++x) {
		for (size_t y = 0; y < ROI.rows; ++y) {
			nowPointer.x = x + bounding.tl().x;
			nowPointer.y = y + bounding.tl().y;
			static bool judge[5];
			for (int i = 0; i < 4; i++) {
				judge[i] = ((A[i] * nowPointer.x + B[i] * nowPointer.y + C[i]) * Flag[i]) > 0;
			}
			judge[4] = _calculatePointsDistance(rect.center, nowPointer)>radiusLimit?true:false;//在圆外
			//不该涂黑
			if (judge[0] && judge[1] && judge[2] && judge[3]&&judge[4])continue;
			//该涂黑
			ROI.at<uchar>(y, x) = 0;
		}
	}
#ifdef BUFFTESTDEBUG
//	cv::imshow("ArmorROIWindow",ROI);
//	cv::waitKey(1);
	//_saveSample->save(ROI);
#endif
	return ROI;
}


size_t Buff::_dealFeature(cv::Mat& img) {
	return cv::countNonZero(img);
}


//画旋转矩形
void Buff::_drawRotatedRect(cv::Mat& img, cv::RotatedRect& rect, int b, int g, int r) {
	cv::Point2f pts[4];
	rect.points(pts);
	cv::line(img, pts[0], pts[1], cv::Scalar(b, g, r), 3, 8);
	cv::line(img, pts[1], pts[2], cv::Scalar(b, g, r), 3, 8);
	cv::line(img, pts[2], pts[3], cv::Scalar(b, g, r), 3, 8);
	cv::line(img, pts[3], pts[0], cv::Scalar(b, g, r), 3, 8);
};


//<---------------------------------------------ROI---------------------------------------------------->
void Buff::_MAKEROI() {
	if (_LASTRFLAG) {
		//if(false){
		cv::Point tl;
		cv::Point br;
		int RWidth = _LASTRRECT.width;
		cv::Point RCenter = _getRectCenter(_LASTRRECT);
		tl = RCenter - cv::Point(15 * RWidth, 15 * RWidth);
		br = RCenter + cv::Point(15 * RWidth, 15 * RWidth);
		if (tl.x < 3)tl.x = 3;
		if (tl.y < 3)tl.y = 3;
		if (br.x > _originMat.cols - 3)br.x = _originMat.cols - 3;
		if (br.y > _originMat.rows - 3)br.y = _originMat.rows - 3;
		if (tl.x < br.x - 20 && tl.y < br.y - 20) {
			cv::Rect ROIRect(tl, br);
			_ROI = _originMat(ROIRect).clone();
			_THISUSEROI = true;
			_THISROIRECT = ROIRect;
		}
		else {
			_ROI = _originMat.clone();
			_THISUSEROI = false;
		}
	}
	else {
		_ROI = _originMat.clone();
		_THISUSEROI = false;
	}
#ifdef BUFFTESTDEBUG
	_ROI.copyTo(_DEBUGMAT);//获取画布
#endif
}


void Buff::_UPDATEROI() {
	if (_findRFlag) {
		_LASTRFLAG = true;
		//这一帧使用了ROI
		if (_THISUSEROI) {
			//_LASTRRECT为R在原图中的矩形
			_LASTRRECT = _RresultRect + _THISROIRECT.tl();
		}
		else {
			_LASTRRECT = _RresultRect;
		}
	}
	else {
		_LASTRFLAG = false;
	}
}
//<---------------------------------------------ROI---------------------------------------------------->



//<-------------------------------------------预判接口------------------------------------------------->
cv::RotatedRect Buff::getTargetRect() {
	cv::RotatedRect targetRect(cv::Point2f(0, 0), cv::Point2f(0, 0), cv::Point2f(0, 0));
	if (_targetArmorIndex == -1)return targetRect;
	if (_findRFlag) {
		targetRect = _Rects[_targetArmorIndex];
		//ROI变换
		if (_THISUSEROI) {//本次使用了ROI
			cv::Point2f tl = _THISROIRECT.tl();
			cv::Point2f fourPoints[4];
			targetRect.points(fourPoints);
			targetRect = cv::RotatedRect(fourPoints[0] + tl, fourPoints[1] + tl, fourPoints[2] + tl);
		}
		//将装甲板缩小一些
	   //targetRect.center.y += 40;
		//targetRect.center.x += 50;
		targetRect = cv::RotatedRect(targetRect.center, cv::Size2f(targetRect.size.width * 0.8f, targetRect.size.height * 0.8f), targetRect.angle);
	}
	return targetRect;
}

cv::RotatedRect Buff::get_R_Rect() {
	cv::RotatedRect Rrect(cv::Point2f(0, 0), cv::Point2f(0, 0), cv::Point2f(0, 0));
	if (_findRFlag) {
		cv::Point2f p = cv::Point2f(_RresultRect.br().x, _RresultRect.tl().y);
		Rrect = cv::RotatedRect(_RresultRect.tl(), p, _RresultRect.br());
		//ROI变换
		if (_THISUSEROI) {//本次使用了ROI
			cv::Point2f tl = _THISROIRECT.tl();
			cv::Point2f fourPoints[4];
			Rrect.points(fourPoints);
			Rrect = cv::RotatedRect(fourPoints[0] + tl, fourPoints[1] + tl, fourPoints[2] + tl);
		}
		//将装甲板缩小一些
	   //targetRect.center.y += 40;
		//targetRect.center.x += 50;
		//Rrect = cv::RotatedRect(Rrect.center, cv::Size2f(Rrect.size.width * 0.8f, Rrect.size.height * 0.8f), Rrect.angle);
	}
	return Rrect;
}

cv::Point Buff::getRCenter() {
	cv::Point R(0, 0);
	if (_findRFlag) {
		R = _RresultRect.tl() + cv::Point(_RresultRect.width / 2, _RresultRect.height / 2);
		if (_THISUSEROI) {
			R = R + _THISROIRECT.tl();
		}
	}
	//R.x += 50;
	//R.y += 40;

	return R;
}
bool Buff::getRFlag() {
	return _findRFlag;
}

bool Buff::getSameTargetFlag() {
	if (_findRFlag && (_targetArmorIndex != -1)) {
		_sameTargetFlag = true;
	}
	else
		_sameTargetFlag = false;
	return _sameTargetFlag;
}
cv::Mat Buff::getDEBUGMAT() {
	return _DEBUGMAT.clone();
}
cv::Mat Buff::getROI() {
	return _ROI.clone();
}
cv::Mat Buff::getBinaryMat() {
	return _binaryMat.clone();
}

//<-------------------------------------------预判接口------------------------------------------------->
