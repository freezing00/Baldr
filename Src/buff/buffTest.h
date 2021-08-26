/*RoboMaster Evolution
* university:GUILIN UNIVERSITY OF ELECTRONIC TECHNOLOGY
**/
#ifndef __BUFFTEST_H__
#define __BUFFTEST_H__
#include<opencv2/opencv.hpp>
#include <tool/Conf.h>
#define BUFF_PI 3.1415926

class _Buff_SaveImage {
private:
	long int _now = 0;//记录现在的文件名称
	std::string* _saveAdress;
public:
	//构造函数
	_Buff_SaveImage(const char* adress);
	//析构函数
	~_Buff_SaveImage();
	//保存
	void save(cv::Mat& img);
};


class Buff {
private:
	//<--------ROI-------->
	bool _LASTRFLAG = false;
	void _MAKEROI();
	void _UPDATEROI();
	bool _THISUSEROI = false;
	cv::Rect _THISROIRECT;
	cv::Rect _LASTRRECT;
	//<--------ROI-------->
	_Buff_SaveImage* _saveSample;//样本保存器
	bool _ownColor;
	bool _sameTargetFlag = false;
	cv::Mat _originMat;
	cv::Mat _ROI;
	cv::Mat _grayMat;
	cv::Mat _binaryMat;
	cv::Rect _RresultRect;
	cv::Mat _DEBUGMAT;
	cv::Mat _separationSrcWhite;
	bool _findRFlag;
	std::vector<cv::Mat> _splitedMats;
	std::vector<std::vector<cv::Point>>_contours;
	std::vector<cv::Vec4i>_hierachy;
	std::vector<cv::RotatedRect>_Rects;
	std::vector<int>_RIndex;
	std::vector<int>_ArmorIndex;
	NewBuffPara _newBuffPara = NewBuffParaFactory::getNewBuff();
	R_Para _r_Para = NewBuffParaFactory::get_R();
	//机器学习 sR识别
	cv::Ptr<cv::ml::ANN_MLP>_Recognizer;
	int _targetArmorIndex;
	void _init(cv::Mat& img, bool ownColor);
	void _preprocess();
	void _findContoursAndRects();
	bool _findR();
	bool _RMorphologicalDiscrimination(cv::RotatedRect& rect);
	float _getRotatedRectSide(cv::RotatedRect& rect, bool flag);
	float _RAIDiscrimination(cv::RotatedRect& rect);
	cv::Mat _makeRImage(cv::RotatedRect& rect);
	void _resizeRect(cv::Rect2f& rect, float u);
	bool _testROI(cv::Rect& rect);
	bool _testROI(cv::Rect2f& rect);
	void _findArmors();
	bool _armorsShapeDetection(cv::RotatedRect& rect);
	cv::Point _getRectCenter(cv::Rect& rect);
	double _calculatePointsDistance(cv::Point2f p1, cv::Point2f p2);
	void _findArmorsROIs();
	cv::Point _getArmorROICenter(cv::Point RCenter, cv::Point ArmorCenter, float u);
	cv::Mat _ArmorROIProcess(cv::RotatedRect rect);
	size_t _dealFeature(cv::Mat& img);
	void _drawRotatedRect(cv::Mat& img, cv::RotatedRect& rect, int b, int g, int r);
	bool _limitArmorAndRAngle(cv::RotatedRect& rect);
	float _calculateLineAndLineAngle(float A_1,float B_1, float A_2, float B_2);
public:
	Buff();
	void setMat(cv::Mat& img, bool ownColor);
	cv::RotatedRect getTargetRect();
	cv::RotatedRect get_R_Rect();
	cv::Point getRCenter();
	bool getRFlag();
	bool getSameTargetFlag();
	cv::Mat getDEBUGMAT();
	cv::Mat getROI();
	cv::Mat getBinaryMat();
};
#endif
