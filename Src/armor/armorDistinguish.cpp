#include "armorDistinguish.h"

//#define SHOW_IMG
//#define USE_SHOW
//#define USE_TRAIN

#ifdef USE_TRAIN
//#define USE_BP
#endif

#ifndef  USE_KCF
#define USE_WEIGHT                //使用权重跟随目标
//#define DEBUG_WEIGHT
#endif

ArmorDistinguish::ArmorDistinguish() {
#ifdef USE_TRAIN
    _standard_2_t = cv::imread("../trainData/standard_2/t.bmp");
    _standard_2_f = cv::imread("../trainData/standard_2/f.bmp");
    cv::resize(_standard_2_t, _standard_2_t, cv::Size(200, 200));
    cv::cvtColor(_standard_2_t, _standard_2_t, cv::COLOR_BGR2GRAY);
    cv::threshold(_standard_2_t, _standard_2_t, 20, 255, 0);
    cv::resize(_standard_2_f, _standard_2_f, cv::Size(200, 200));
    cv::cvtColor(_standard_2_f, _standard_2_f, cv::COLOR_BGR2GRAY);
    cv::threshold(_standard_2_f, _standard_2_f, 20, 255, 0);
#ifdef USE_BP
    //net = cv::dnn::readNetFromCaffe("../trainData/deploy.prototxt", "../trainData/_iter_2000.caffemodel");
    _net.load("../trainData/model_sigmoid_800_200.xml");
#else
    //_svmHog = cv::Algorithm::load<cv::ml::SVM>("../trainData/model_liner2.xml");
#endif
#endif
}

#ifdef USE_KCF
//kcf追踪器
void ArmorDistinguish::armorTrackerInit(const cv::Mat& src, EnemyColor enemyColor) {
    cv::Mat img;
    _size = src.size();
    _para.enemyColor = enemyColor;
    _params.detect_thresh = 0.03f;
    if (_isTrackerStart) {
        //追踪失败
        src(_trackerRect).copyTo(img);
        if (!_tracker->update(_src, _trackerRect)) {
            _trackerSuccess = false;
            _searchRect.x = _trackerRect.x - _trackerRect.width;
            _searchRect.y = _trackerRect.y - _trackerRect.height;
            _searchRect.height = _trackerRect.height * 3;
            _searchRect.width = _trackerRect.width * 3;
            _searchRect &= cv::Rect2d(0, 0, _imgWidth, _imgHeight);
        }
        else {
            //越界则需要从全图搜索
            if ((_trackerRect & cv::Rect2d(0, 0, _imgWidth, _imgHeight)) != _trackerRect) {
                _searchRect = cv::Rect2d(0, 0, _imgWidth, _imgHeight);
            }
            //搜索区域需要扩大两倍
            else {
                _trackerSuccess = true;
                _searchRect.x = _trackerRect.x - _trackerRect.width / 2;
                _searchRect.y = _trackerRect.y - _trackerRect.height / 2;
                _searchRect.height = _trackerRect.height * 2;
                _searchRect.width = _trackerRect.width * 2;
                _searchRect &= cv::Rect2d(0, 0, _imgWidth, _imgHeight);
            }
        }
        src(_searchRect).copyTo(_src);
        imshow("img", img);
    }
    else {
        _trackerSuccess = false;
        _searchRect = cv::Rect2d(0, 0, _imgWidth, _imgHeight);
        _src = src;
    }

    imshow("_src", _src);
    cv::waitKey(1);
}
#endif

//识别前的图片准备
void ArmorDistinguish::imagePreprocess(const cv::Mat& src, EnemyColor enemyColor) {

    _para.enemyColor = enemyColor;
    const cv::Point& lastResult = _resLast.center;                         //上一个运行结果取上一次方框的中心
    if (lastResult.x == 0 || lastResult.y == 0) {                          //如果上一次结果的x或y为0，就以整个图片的大小作方框
        _src = src;                                                        //继承原图片地址
        _restoreRect = cv::Rect(0, 0, src.cols, src.rows);
        _firstEnterFlag = true;
    }
    else {                                                                //否则继承上一次的图片大小
        _firstEnterFlag = false;
        cv::Rect rect = _resLast.boundingRect();                            //返回包含旋转矩形的最小右上整数矩形。
        static float ratioToWidth = 4.8f;
        static float ratioToHeight = 3.0f;
        int x1 = MAX(int(lastResult.x - (rect.width * ratioToWidth)), 0);
        int y1 = MAX(int(lastResult.y - (rect.height * ratioToHeight)), 0);
        cv::Point lu = cv::Point(x1, y1);                                                           //得到点lu，ROI左上角的起点
        int x2 = MIN(int(lastResult.x + (rect.width * ratioToWidth)), src.cols);                //计算结果不大于图片长宽
        int y2 = MIN(int(lastResult.y + (rect.height * ratioToHeight)), src.rows);
        cv::Point rd = cv::Point(x2, y2);                                                           //得到点rd，ROI右下角的终点
        _restoreRect = cv::Rect(lu, rd);
        src(_restoreRect).copyTo(_src);
    }

    //图像二值化
    cv::split(_src, splitSrc);                                                               //分离色彩通道
    cv::cvtColor(_src, _graySrc, cv::COLOR_BGR2GRAY);                                        //获取灰度图
    cv::threshold(_graySrc, _separationSrcWhite, 240, 255, cv::THRESH_BINARY);
    cv::bitwise_not(_separationSrcWhite, _separationSrcWhite);

    if (enemyColor == ENEMY_RED) {
        //敌方为红色
        cv::threshold(_graySrc, _graySrc, _para.grayThreshold_RED, 255, cv::THRESH_BINARY);     //灰度二值化
        cv::subtract(splitSrc[2], splitSrc[0], _separationSrc);                                 //红蓝通道相减
        cv::subtract(splitSrc[2], splitSrc[1], _separationSrcGreen);                             //红绿通道相减
        cv::threshold(_separationSrc, _separationSrc, _para.separationThreshold_RED, 255, cv::THRESH_BINARY);             //红蓝二值化
        cv::threshold(_separationSrcGreen, _separationSrcGreen, _para.separationThreshold_GREEN, 255, cv::THRESH_BINARY);//红绿二值化
        cv::dilate(_separationSrc, _separationSrc, Util::structuringElement3());
        cv::dilate(_separationSrcGreen, _separationSrcGreen, Util::structuringElement3());                                        //膨胀

        _maxColor = _separationSrc & _graySrc & _separationSrcGreen & _separationSrcWhite;                                                                //逻辑与获得最终二值化图像
        cv::dilate(_maxColor, _maxColor, Util::structuringElement3());                                                    //膨胀
        //cv::morphologyEx(_maxColor, _maxColor, cv::MORPH_OPEN, Util::structuringElement3());
    }
    else {
        cv::threshold(splitSrc[2], _purpleSrc, _para.grayThreshold_PURPLE, 255, cv::THRESH_BINARY);                 //防止误识别紫色基地
        cv::bitwise_not(_purpleSrc, _purpleSrc);
        //敌方为蓝色
        cv::threshold(_graySrc, _graySrc, _para.grayThreshold_BLUE, 255, cv::THRESH_BINARY);                        //灰度二值化
        cv::subtract(splitSrc[0], splitSrc[2], _separationSrc);
        cv::subtract(splitSrc[0], splitSrc[1], _separationSrcGreen);                                                //红蓝通道相减
        cv::threshold(_separationSrc, _separationSrc, _para.separationThreshold_BLUE, 255, cv::THRESH_BINARY);
        cv::threshold(_separationSrcGreen, _separationSrcGreen, _para.separationThreshold_GREEN, 255, cv::THRESH_BINARY);//红蓝二值化
        cv::dilate(_separationSrc, _separationSrc, Util::structuringElement3());
        cv::dilate(_separationSrcGreen, _separationSrcGreen, Util::structuringElement3());           //膨胀

        _maxColor = _separationSrc & _graySrc & _separationSrcGreen & _separationSrcWhite & _purpleSrc;                                                                //逻辑与获得最终二值化图像
        cv::dilate(_maxColor, _maxColor, Util::structuringElement3());                                                    //膨胀
        //cv::morphologyEx(_maxColor, _maxColor, cv::MORPH_OPEN, Util::structuringElement3());
    }
}


//找到发光体轮廓
void ArmorDistinguish::findLightBarContour() {
    //找到灯条轮廓
    cv::findContours(_maxColor, allContours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //直接通过轮廓筛选  然后才通过拟合的旋转矩形筛选
    for (int i = 0; i < allContours.size(); ++i) {
        if (hierarchy[i][3] == -1) {
            cv::RotatedRect scanRect = cv::minAreaRect(allContours[i]);                    //检测最小面积的矩形

            cv::Point2f vertices[4];
            scanRect.points(vertices);

            if (fabs(vertices[1].x - vertices[3].x) > fabs(vertices[1].y - vertices[3].y))
                continue;
            //std::cout << "scanRect.size.area():" << scanRect.size.area() << std::endl;
            if (scanRect.size.area() < _para.minLightBarArea)
                continue;

            //rect的高度、和宽度有一个小于板灯的最小高度就直接跳过本次循环
            float longSide = Util::findExtremumOfSide(scanRect, LONG_SIDE);
            float shortSide = Util::findExtremumOfSide(scanRect, SHORT_SIDE);
            //std::cout << "longSide:" << longSide << std::endl;
            //std::cout << "shortSide:" << shortSide << std::endl;
            if (longSide > _para.maxLightBarLength || longSide < _para.minLightBarLength || shortSide > _para.maxLightBarWidth || shortSide < _para.minLightBarWidth)
                continue;

            if (longSide > shortSide * 8.0f || longSide < shortSide * 1.5f)
                continue;

            possibleRects.push_back(scanRect);
            //std::cout << "*" << std::endl;
        }
    }
}


//对发光体进行筛选,将符合条件的灯条配对
void ArmorDistinguish::lightBarFilter() {
    //std::cout << inputRects.size() << std::endl;
    if (possibleRects.size() < 2) {
        return;
    }
    float centre_between = 0, scale_max = 6;
    cv::Point2f shortCenter1[2];
    cv::Point2f shortCenter2[2];
    for (size_t i = 0; i < possibleRects.size() - 1; ++i) {
        for (size_t j = i + 1; j < possibleRects.size(); ++j) {
            float maxLength_1 = 0, maxLength_2 = 0, maxLength_between = 0, minLength_between;
            maxLength_1 = Util::findExtremumOfSide(possibleRects[i], LONG_SIDE);
            maxLength_2 = Util::findExtremumOfSide(possibleRects[j], LONG_SIDE);
            maxLength_between = maxLength_1 > maxLength_2 ? maxLength_1 : maxLength_2;
            minLength_between = maxLength_1 < maxLength_2 ? maxLength_1 : maxLength_2;
            if (fabs(possibleRects[i].center.x - possibleRects[j].center.x) > maxLength_between ||
                fabs(possibleRects[i].center.y - possibleRects[j].center.y) > minLength_between * 0.3f) {
                //最长的一边也必须在一定的长度范围内,面积也要在一定范围内
                float heightScale = maxLength_1 > maxLength_2 ? (maxLength_1 / maxLength_2) : (maxLength_2 / maxLength_1);
                if (heightScale < 1.4f) {
                    centre_between = sqrtf((float)pow(possibleRects[i].center.x - possibleRects[j].center.x, 2) + (float)pow(possibleRects[i].center.y - possibleRects[j].center.y, 2));
                    //限制装甲矩形的长宽比
                    if (centre_between < maxLength_between * scale_max) {
                        getShortCenter(possibleRects[i], shortCenter1);
                        getShortCenter(possibleRects[j], shortCenter2);

                        float tan1 = 0.0f, tan2 = 0.0f;
                        cv::Point2f vertices1[4];
                        cv::Point2f vertices2[4];
                        possibleRects[i].points(vertices1);
                        possibleRects[j].points(vertices2);

                        if (fabs(vertices1[0].x - vertices1[2].x) > fabs(vertices1[1].x - vertices1[3].x)) {
                            if (vertices1[0].y > vertices1[2].y) {
                                tan1 = (vertices1[0].y - vertices1[2].y) / vertices1[0].x - vertices1[2].x;
                            }
                            else {
                                tan1 = (vertices1[2].y - vertices1[0].y) / vertices1[2].x - vertices1[0].x;
                            }
                        }
                        else {
                            if (vertices1[1].y > vertices1[3].y) {
                                tan1 = (vertices1[1].y - vertices1[3].y) / vertices1[1].x - vertices1[3].x;
                            }
                            else {
                                tan1 = (vertices1[3].y - vertices1[1].y) / vertices1[3].x - vertices1[1].x;
                            }
                        }

                        if (fabs((vertices2[0].y - vertices2[2].y) / (vertices2[0].x - vertices2[2].x)) > fabs((vertices2[1].y - vertices2[3].y) / (vertices2[1].x - vertices2[3].x))) {
                            if (vertices2[0].y > vertices2[2].y) {
                                tan2 = (vertices2[0].y - vertices2[2].y) / vertices2[0].x - vertices2[2].x;
                            }
                            else {
                                tan2 = (vertices2[2].y - vertices2[0].y) / vertices2[2].x - vertices2[0].x;
                            }
                        }
                        else {
                            if (vertices2[1].y > vertices2[3].y) {
                                tan2 = (vertices2[1].y - vertices2[3].y) / vertices2[1].x - vertices2[3].x;
                            }
                            else {
                                tan2 = (vertices2[3].y - vertices2[1].y) / vertices2[3].x - vertices2[1].x;
                            }
                        }

                        float lightAngle = Util::lineToLineAngle(shortCenter1[0], shortCenter1[1], shortCenter2[0], shortCenter2[1]);

                        if (tan1 * tan2 < 0.0f)
                            continue;

                        //两个灯条矩形的角度差都不能小于一定值
                        if (lightAngle > 13.0f)
                            continue;

                        if (calSkewingAngle(possibleRects[i], possibleRects[j], shortCenter1) < 75.0f || calSkewingAngle(possibleRects[i], possibleRects[j], shortCenter2) < 75.0f)
                            continue;

                        //std::cout << "lineToLineAngle<13:" << Util::lineToLineAngle(shortCenter1[0], shortCenter1[1], shortCenter2[0], shortCenter2[1]) << std::endl;
                        //std::cout << "calSkewingAngle<75:" << calSkewingAngle(inputRects[i], inputRects[j], shortCenter1) << std::endl;

                        cv::RotatedRect armor = Util::boundingRRect(possibleRects[i], possibleRects[j]);
                        if(fabs(armor.angle) > 10.0f && fabs(armor.angle) < 170.0f)
                            continue;

                        ArmorStruct armorStruct;
                        armorStruct.lightAngle = lightAngle;
                        armorStruct.partLightBars[0] = (possibleRects[i].center.x < possibleRects[j].center.x ? possibleRects[i] : possibleRects[j]);
                        armorStruct.partLightBars[1] = (possibleRects[i].center.x > possibleRects[j].center.x ? possibleRects[i] : possibleRects[j]);
                        armorStruct.armorRect = armor;
                        distinguishArmorType(armorStruct);
                        armorStructs.push_back(armorStruct);
                    }
                }
            }
        }
    }
    //std::cout << "armorStructs:" << armorStructs.size() << std::endl;
}

void ArmorDistinguish::getShortCenter(cv::RotatedRect rect, cv::Point2f* shortCenter) {
    if (rect.size.width <= 0) {
        return;
    }
    cv::Point2f verts[4];
    rect.points(verts);
    if (Util::pointDistance(verts[0], verts[1]) < Util::pointDistance(verts[1], verts[2])) {
        shortCenter[0] = (verts[0] + verts[1]) / 2;
        shortCenter[1] = (verts[2] + verts[3]) / 2;
    }
    else {
        shortCenter[0] = (verts[1] + verts[2]) / 2;
        shortCenter[1] = (verts[3] + verts[0]) / 2;
    }
}

float ArmorDistinguish::calSkewingAngle(cv::RotatedRect rect1, cv::RotatedRect rect2, cv::Point2f* shortCenter) {
    if (rect1.size.width <= 0 || rect1.size.height <= 0) {
        return float();
    }
    return Util::lineToLineAngle(rect1.center, rect2.center, shortCenter[0], shortCenter[1]);
}

//区分装甲板类型
void ArmorDistinguish::distinguishArmorType(ArmorStruct& armorStructs) {
    float longSide_1 = Util::findExtremumOfSide(armorStructs.partLightBars[0], LONG_SIDE);
    float longSide_2 = Util::findExtremumOfSide(armorStructs.partLightBars[1], LONG_SIDE);
    float heightScale = longSide_1 > longSide_2 ? (longSide_1 / longSide_2) : (longSide_2 / longSide_1);

    //std::cout << "getRectLengthWidthRatio:" << Util::getRectLengthWidthRatio(armorStructs.armorRect) << std::endl;
    //std::cout << "heightScale:" << heightScale << std::endl;

    if (Util::getRectLengthWidthRatio(armorStructs.armorRect) > 3.3f) {
        _bigArmorFlag = true;
        armorStructs.armorType = 1;
    }
    else if (Util::getRectLengthWidthRatio(armorStructs.armorRect) > 2.6f) {
        if (heightScale > 1.3f) {
            _bigArmorFlag = true;
            armorStructs.armorType = 1;
        }
        else {
            armorStructs.armorType = 0;
        }
    }
    else armorStructs.armorType = 0;
}

//找深度信息最小的装甲板;armorType(0是小，1是大)
void ArmorDistinguish::findNearestArmor(std::vector<ArmorStruct>& armorStructs, cv::RotatedRect& resultRect) {
    if (armorStructs.empty()) {
        return;
    }

    if (armorStructs.size() == 1) {
        resultRect = armorStructs[0].armorRect;
        _leftLightBar = armorStructs[0].partLightBars[0];
        _rightLightBar = armorStructs[0].partLightBars[1];
        _armorType = armorStructs[0].armorType;
        //_nowCarID = armorStructs[0].carId;
        return;
    }

    //cv::Point2f centerVision((float)_src.cols / 2, (float)_src.rows / 2);
    vector<ArmorStruct>::iterator it;
    ArmorStruct armorStruct = armorStructs[0];
    float maxArea = armorStructs[0].armorRect.size.area(); 
    float minDistance = 1920;
    float area = 0.0f;
    float distance = 0.0f;
    cv::Point2f middle = cv::Point2f(0, 356.2);
    for (it = armorStructs.begin(); it != armorStructs.end(); it++) {
        if (_carType == NEW_SENTRY_ABOVE || _carType == NEW_SENTRY_BELOW) {
            //循环替换最近距离同时替换装甲板结构体
            area = (*it).armorRect.size.area();
            if (area > maxArea) {
                maxArea = area;
                armorStruct = (*it);
            }
        }
        else {
            distance = Util::pointDistance(middle, (*it).armorRect.center);
            if (distance < minDistance) {
                minDistance = distance;
                armorStruct = (*it);
            }
        }
    }
    resultRect = armorStruct.armorRect;
    _leftLightBar = armorStruct.partLightBars[0];
    _rightLightBar = armorStruct.partLightBars[1];
    _armorType = armorStruct.armorType;
    //_nowCarID = armorStructs[0].carId;

    return;
}

void ArmorDistinguish::removeWrongArmor() {
    float centreDistance[2];

    //去除含有内嵌灯条的装甲板
    for (int i = 0; i < armorStructs.size(); ++i) {
        for (int j = 0; j < possibleRects.size(); ++j) {
            centreDistance[0] = Util::pointDistance(armorStructs[i].partLightBars[1].center, possibleRects[j].center);
            centreDistance[1] = Util::pointDistance(armorStructs[i].partLightBars[0].center, possibleRects[j].center);
            if (centreDistance[0] && centreDistance[1]) {
                if (embeddedRectJudge(armorStructs[i].armorRect, possibleRects[j])) {
                    armorStructs.erase(std::begin(armorStructs) + i);
                    break;
                }
            }
        }
    }

    if (armorStructs.size() <= 1) {
        return;
    }

    //有装甲板公用同一灯条，去除长宽比大的或夹角大的
    for (int i = 0; i < armorStructs.size() - 1; ++i) {
        for (int j = i + 1; j < armorStructs.size(); ++j) {
            centreDistance[0] = Util::pointDistance(armorStructs[i].partLightBars[1].center, armorStructs[j].partLightBars[0].center);
            centreDistance[1] = Util::pointDistance(armorStructs[i].partLightBars[0].center, armorStructs[j].partLightBars[1].center);
            if (!centreDistance[0]) {

                if(armorStructs[i].armorType == true && armorStructs[j].armorType == true) {
                    if (armorStructs[i].lightAngle > armorStructs[j].lightAngle) {
                        armorStructs.erase(std::begin(armorStructs) + i);
                    }
                    else if (armorStructs[i].lightAngle < armorStructs[j].lightAngle) {
                        armorStructs.erase(std::begin(armorStructs) + j);
                    } else {
                       if (armorStructs[i].armorRect.size.width/armorStructs[i].armorRect.size.height > armorStructs[j].armorRect.size.width/armorStructs[j].armorRect.size.height) {
                           armorStructs.erase(std::begin(armorStructs) + i);
                       } else {
                           armorStructs.erase(std::begin(armorStructs) + j);
                       }
                    }
                } else {
                    if (armorStructs[i].armorRect.size.width/armorStructs[i].armorRect.size.height > armorStructs[j].armorRect.size.width/armorStructs[j].armorRect.size.height) {
                        armorStructs.erase(std::begin(armorStructs) + i);
                    } else {
                        armorStructs.erase(std::begin(armorStructs) + j);
                    }
                }
            }
            else if (!centreDistance[1]) {


                if(armorStructs[i].armorType == true && armorStructs[j].armorType == true){
                    if (armorStructs[i].lightAngle > armorStructs[j].lightAngle) {
                        armorStructs.erase(std::begin(armorStructs) + i);
                    }
                    else if (armorStructs[i].lightAngle < armorStructs[j].lightAngle) {
                        armorStructs.erase(std::begin(armorStructs) + j);
                    } else {
                        if (armorStructs[i].armorRect.size.width/armorStructs[i].armorRect.size.height > armorStructs[j].armorRect.size.width/armorStructs[j].armorRect.size.height) {
                            armorStructs.erase(std::begin(armorStructs) + i);
                        } else {
                            armorStructs.erase(std::begin(armorStructs) + j);
                        }
                    }
                } else {
                    if (armorStructs[i].armorRect.size.width/armorStructs[i].armorRect.size.height > armorStructs[j].armorRect.size.width/armorStructs[j].armorRect.size.height) {
                        armorStructs.erase(std::begin(armorStructs) + i);
                    } else {
                        armorStructs.erase(std::begin(armorStructs) + j);
                    }
                }
            }
        }
    }
}

//根据权重找装甲板
void ArmorDistinguish::findArmorByWeight(std::vector<ArmorStruct>& armorStructs, cv::RotatedRect& resultRect, ArmorRectHistoricalDataList& armorRectData) {
    //float aver = 0.0f;
    for (size_t i = 0; i < armorStructs.size(); ++i) {
        float rectsArea = armorStructs[i].armorRect.size.height * armorStructs[i].armorRect.size.width;
        armorRectData.areaChangeRate.push_back(fabs(rectsArea - _armorRectHistoricalData.area.back()) / rectsArea);
        armorRectData.areaChangeRateChangeRate.push_back(fabs(armorRectData.areaChangeRate.back() - _armorRectHistoricalData.areaChangeRate.back()) / rectsArea);
        float rectsLengthWidthRatio = Util::getRectLengthWidthRatio(armorStructs[i].armorRect);
        armorRectData.lenWidChangeRate.push_back(fabs(rectsLengthWidthRatio - _armorRectHistoricalData.lengthWidthRatio.back()));
        cv::Point2f nowCenter2D = cv::Point2f(armorStructs[i].armorRect.center.x + _restoreRect.x, armorStructs[i].armorRect.center.y + _restoreRect.y);
        armorRectData.xChangeRate.push_back(fabsf(nowCenter2D.x - _armorRectHistoricalData.x.back()) / rectsArea);
        armorRectData.yChangeRate.push_back(fabsf(nowCenter2D.y - _armorRectHistoricalData.y.back()) / rectsArea);
    }
    maxWeightNum = findVarianceMaxWeight(_armorRectHistoricalData, armorRectData, _maxWeightValue);
    resultRect = armorStructs[maxWeightNum].armorRect;
    //判断目标装甲板类型
    _armorType = armorStructs[maxWeightNum].armorType;
    _leftLightBar = armorStructs[0].partLightBars[0];
    _rightLightBar = armorStructs[0].partLightBars[1];
    _nowCarID = armorStructs[maxWeightNum].carId;
}

//通过ml检测是否为装甲板
void ArmorDistinguish::armorChooseTarget(EnemyColor enemyColor) {
    if (armorStructs.empty()) {
        return;
    }
    std::vector<ArmorStruct>::iterator it = armorStructs.begin();
    while (it != armorStructs.end()) {

        if (!getBlobColor(enemyColor, (*it).partLightBars[0], (*it).partLightBars[1])) {
            armorStructs.erase(it);
            continue;
        }
#ifdef USE_TRAIN
        if (_carType == NEW_SENTRY_BELOW || _carType == NEW_SENTRY_ABOVE) {
            cv::RotatedRect& rects = (*it).armorRect;
            float longSide = Util::findExtremumOfSide(rects, LONG_SIDE);
            //float shortSide = Util::findExtremumOfSide(rects, SHORT_SIDE);

            //获取装甲板矩形的最小外接矩形
            cv::Rect bounding_roi = rects.boundingRect();
            //根据不同的长宽比来切换矩形大小
            if (bounding_roi.width < bounding_roi.height) {
                cv::Point centre((bounding_roi.tl().x + bounding_roi.br().x) / 2, (bounding_roi.tl().y + bounding_roi.br().y) / 2);
                cv::Point half(bounding_roi.height / 2, bounding_roi.width / 2);
                bounding_roi = cv::Rect(centre - half, centre + half);                                    //重新定位tl和br
            }
            //都加上长的一边是为了保证仿射变换后有完整的图进行检测
            bounding_roi.x -= (int)(longSide / 2);
            bounding_roi.width += (int)longSide;
            bounding_roi.y -= (int)(longSide / 2);
            bounding_roi.height += (int)longSide;
            MakeSafeRect(bounding_roi, _src.size());

            cv::Point2f new_center = rects.center - cv::Point2f((float)bounding_roi.x, (float)bounding_roi.y);    //计算新的中心点
            cv::Mat roi_src = _src(bounding_roi);
            cv::Mat rotation = cv::getRotationMatrix2D(new_center, rects.angle, 1);                            //得到旋转后的图
            cv::Mat rectify_target;
            cv::warpAffine(roi_src, rectify_target, rotation, bounding_roi.size());                        //做一次仿射变换

            cv::waitKey(1);

            int minInsert = MIN(rectify_target.rows, rectify_target.cols);                                //选最小的一边，圈一个正方形的ROI
            cv::Point tl(rectify_target.cols / 2 - minInsert / 4, rectify_target.rows / 2 - minInsert / 4);
            cv::Point br(rectify_target.cols / 2 + minInsert / 4, rectify_target.rows / 2 + minInsert / 4);
            cv::Rect square = cv::Rect(tl, br);
            cv::Mat square_target = rectify_target(square);
            cv::Mat resizeImg;
            cv::resize(square_target, resizeImg, cv::Size(200, 200), 0, 0);                                        //重新装载大小
            cv::cvtColor(resizeImg, _bpImg, cv::COLOR_BGR2GRAY);                                                                        //前哨站空白区域
            cv::threshold(_bpImg, _bpImg, 0, 255, cv::THRESH_OTSU);                                                 //识别2号装甲板二值化

            setSampleCaptureState(true);
            double sum1 = 0, sum2 = 0;
            for (int i = 0; i < 200; i++) {
                for (int j = 0; j < 200; j++) {
                    if (_standard_2_t.at<uchar>(i, j) == _bpImg.at<uchar>(i, j)) {
                        sum1++;
                    }
                    if (_standard_2_f.at<uchar>(i, j) == _bpImg.at<uchar>(i, j)) {
                        sum2++;
                    }
                }
            }
            float ratio1 = sum1 * 1.0 / 40000;
            float ratio2 = sum2 * 1.0 / 40000;
            bool  is_2 = 0;
            if (ratio1 > 0.8 || ratio2 > 0.8) {
                is_2 = 1;
                armorStructs.erase(it);
                continue;
            }
            else {
                //std::cout << "不是2号装甲板" << std::endl;
            }
        }
#if USE_CAFFE
        //Mat inputBlob = dnn::blobFromImage(resizeImg, 0.00390625f, Size(20, 20), Scalar(), false);
        Mat inputBlob = dnn::blobFromImage(resizeImg, 1, Size(20, 20), Scalar(), false);
        net.setInput(inputBlob, "data");
        inputBlob = net.forward("loss");
        int classId;
        double classProb;
        inputBlob = inputBlob.reshape(1, 1);
        Point classNumber;
        cv::minMaxLoc(inputBlob, NULL, &classProb, NULL, &classNumber);
        classId = classNumber.x;    // 类别：0是45度，1是装甲板
//        LOG::info("case of armor: " + to_string(classId) + " accuracy: " + to_string(classProb * 100));
        if (classId == 0 || classProb < 0.6) {
            _sampleData.classifyState = false;
            armorStructs.erase(it);
            continue;
        }
#elseif USE_BP
            // cv::Mat grayResizeImg;
            // cv::cvtColor(resizeImg, grayResizeImg, cv::COLOR_RGB2GRAY);
            // cv::Mat predict_img = cv::Mat(400, 1, CV_32FC1);
            // predict_img = bp::Net::bpImgPreProcess(grayResizeImg);
            // int response = _net.predict_one(predict_img);
            // if (response == 0) {
            //     _sampleData.classifyState = false;
            //     armorStructs.erase(it);
            //     continue;
            // }
            // (*it).carId = response;
            // if (response == 1 || response == 7) {
            //     (*it).armorType = 1;
            //     _bigArmorFlag = true;
            // }
            // //cout << response << endl;
            // _sampleData.classifyState = true;
            //int response = armorHogSvm(bpImg);
            //if ((*it).armorType == 1 && (response != 1 || response != 7)) {
            //    _sampleData.classifyState = false;
            //    armorStructs.erase(it);
            //    continue;
            //}
            //(*it).carId = response;
            //std::string strttt[] = { "0","hero      ","engineer","infantry3","infantry4","5","base and skirmish","sentinel" };
            //cout << strttt[response] << endl;
#endif
#endif
        ++it;
    }
}

bool ArmorDistinguish::getBlobColor(EnemyColor enemyColor, const cv::RotatedRect& blobPos1, const cv::RotatedRect& blobPos2) {

    if (blobPos1.size.empty() || blobPos2.size.empty()) {
        return false;
    }

    cv::Rect region1 = blobPos1.boundingRect();
    //cv::Rect region2 = blobPos2.boundingRect();


    Util::makeRectSafe(region1, _src.size());

    cv::Mat light1 = _src(region1);
    //cv::Mat light2 = src(region2);

    float redCnt = 0, blueCnt = 0;
    for (int row = 0; row < light1.rows; row++) {
        for (int col = 0; col < light1.cols; col++) {
            redCnt += light1.at<cv::Vec3b>(row, col)[2];
            blueCnt += light1.at<cv::Vec3b>(row, col)[0];
        }
    }

    //std::cout << "blueCnt/redCnt:" << blueCnt/redCnt << std::endl;

    if (enemyColor == ENEMY_RED) {
        if (redCnt > blueCnt * 1.6f) {
            return true;
        }
        else {
            return false;
        }
    }
    else {
        if (blueCnt > redCnt * 1.6f) {
            return true;
        }
        else {
            return false;
        }
    }
}

bool ArmorDistinguish::embeddedRectJudge(cv::RotatedRect r1, cv::RotatedRect r2) {
    double distance = r1.size.width > r1.size.height ? r1.size.width / 2 : r1.size.height / 2;

    return (Util::pointDistance(r1.center, r2.center) < distance);
}

//对样本进行HOG特征提取返回预测结果
int ArmorDistinguish::armorHogSvm(cv::Mat& inputMat) {
    cv::Mat dst;
    cv::Size matSize = cv::Size(128, 128);
    vector<float> descriptor;
    cv::resize(inputMat, dst, matSize);
    cv::HOGDescriptor myHog = cv::HOGDescriptor(matSize, cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9);
    myHog.compute(dst.clone(), descriptor, cv::Size(1, 1), cv::Size(0, 0));
    cv::Mat dectDescriptor = cv::Mat::zeros(1, descriptor.size(), CV_32FC1);
    size_t len = descriptor.size();
    for (size_t i = 0; i < len; i++) {
        dectDescriptor.at<float>(0, i) = descriptor[i];
    }
    float response = _svmHog->predict(dectDescriptor);
    return (int)response;
}

//装甲板协方差权重计算
float ArmorDistinguish::armorVarianceWeightCalculate(ArmorRectHistoricalDataList& data, size_t i) {
    return (1 - (data.areaChangeRateChangeRateVarianceList[i] * _armorWeightCalculateCoefficient.areaVarianceCoefficient + data.xChangeRateVarianceList[i] * _armorWeightCalculateCoefficient.xVarianceCoefficient  \
        + data.yChangeRateVarianceList[i] * _armorWeightCalculateCoefficient.yVarianceCoefficient + data.lenWidChangeRateVarianceList[i] * _armorWeightCalculateCoefficient.lenWidVarianceCoefficient));

}

//找到权重最大的装甲板
int ArmorDistinguish::findVarianceMaxWeight(ArmorRectHistoricalDataList& lastArmorRectData, struct ArmorRectHistoricalDataList& nowArmorRectData, float& weight) {
    int j = 0;
    armorRectListVarianceDataCalculate(lastArmorRectData, nowArmorRectData, 0);
    float max = armorVarianceWeightCalculate(nowArmorRectData, 0);
    for (size_t i = 1; i < nowArmorRectData.areaChangeRate.size(); ++i) {
        armorRectListVarianceDataCalculate(lastArmorRectData, nowArmorRectData, i);
        weight = armorVarianceWeightCalculate(nowArmorRectData, i);
        if (weight >= max) {
            max = weight;
            j = (int)i;
        }
    }
    weight = max;
    return j;
}

//历史装甲板数据重置
void ArmorDistinguish::armorRectListDataReset(struct ArmorRectHistoricalDataList& armorRectData) {
    armorRectData.area.clear();
    armorRectData.lengthWidthRatio.clear();
    armorRectData.x.clear();
    armorRectData.y.clear();
    armorRectData.areaChangeRate.clear();
    armorRectData.lenWidChangeRate.clear();
    armorRectData.xChangeRate.clear();
    armorRectData.yChangeRate.clear();
    armorRectData.areaChangeRate.push_back(0.0f);
    armorRectData.areaChangeRateChangeRate.push_back(0.0f);
    armorRectData.lenWidChangeRate.push_back(0.0f);
    armorRectData.xChangeRate.push_back(0.0f);
    armorRectData.yChangeRate.push_back(0.0f);
    armorRectData.areaChangeRateChangeRateVarianceList.push_back(0.0f);
    armorRectData.lenWidChangeRateVarianceList.push_back(0.0f);
    armorRectData.xChangeRateVarianceList.push_back(0.0f);
    armorRectData.yChangeRateVarianceList.push_back(0.0f);
}

//历史装甲板基础数据装填
void ArmorDistinguish::armorRectListBaseDataIndex(struct ArmorRectHistoricalDataList& armorRectData, cv::RotatedRect& rect) {
    cv::Point2f resultCenter = cv::Point2f(rect.center.x + _restoreRect.x, rect.center.y + _restoreRect.y);
    if (armorRectData.area.size() < FIFO_LEN) {
        //填充新数据
        armorRectData.area.push_back(rect.size.height * rect.size.width);
        armorRectData.lengthWidthRatio.push_back(Util::getRectLengthWidthRatio(rect));
        armorRectData.x.push_back(resultCenter.x);
        armorRectData.y.push_back(resultCenter.y);
    }
    else {
        //擦除第一个数据
        armorRectData.area.erase(std::begin(armorRectData.area));
        armorRectData.lengthWidthRatio.erase(std::begin(armorRectData.lengthWidthRatio));
        armorRectData.x.erase(std::begin(armorRectData.x));
        armorRectData.y.erase(std::begin(armorRectData.y));
        //填充新数据
        armorRectData.area.push_back(rect.size.height * rect.size.width);
        armorRectData.lengthWidthRatio.push_back(Util::getRectLengthWidthRatio(rect));
        armorRectData.x.push_back(resultCenter.x);
        armorRectData.y.push_back(resultCenter.y);
    }
}

//历史装甲板偏差数据装填
void ArmorDistinguish::armorRectListBiasDataIndex(struct ArmorRectHistoricalDataList& lastArmorRectData, struct ArmorRectHistoricalDataList& nowArmorRectData, uint8_t i, uint8_t mode) {

    if (lastArmorRectData.areaChangeRate.size() < FIFO_LEN) {
        //填充新数据
        lastArmorRectData.areaChangeRate.push_back(nowArmorRectData.areaChangeRate[i]);
        lastArmorRectData.areaChangeRateChangeRate.push_back(nowArmorRectData.areaChangeRateChangeRate[i]);
        lastArmorRectData.lenWidChangeRate.push_back(nowArmorRectData.lenWidChangeRate[i]);
        lastArmorRectData.xChangeRate.push_back(nowArmorRectData.xChangeRate[i]);
        lastArmorRectData.yChangeRate.push_back(nowArmorRectData.yChangeRate[i]);

    }
    else {
        //擦除第一个数据
        lastArmorRectData.areaChangeRate.erase(std::begin(lastArmorRectData.areaChangeRate));
        lastArmorRectData.areaChangeRateChangeRate.erase(std::begin(lastArmorRectData.areaChangeRateChangeRate));
        lastArmorRectData.lenWidChangeRate.erase(std::begin(lastArmorRectData.lenWidChangeRate));
        lastArmorRectData.xChangeRate.erase(std::begin(lastArmorRectData.xChangeRate));
        lastArmorRectData.yChangeRate.erase(std::begin(lastArmorRectData.yChangeRate));
        //填充新数据
        lastArmorRectData.areaChangeRate.push_back(nowArmorRectData.areaChangeRate[i]);
        lastArmorRectData.areaChangeRateChangeRate.push_back(nowArmorRectData.areaChangeRateChangeRate[i]);
        lastArmorRectData.lenWidChangeRate.push_back(nowArmorRectData.lenWidChangeRate[i]);
        lastArmorRectData.xChangeRate.push_back(nowArmorRectData.xChangeRate[i]);
        lastArmorRectData.yChangeRate.push_back(nowArmorRectData.yChangeRate[i]);
    }
}

//装甲板方差数据装填
void ArmorDistinguish::armorRectListVarianceDataCalculate(struct ArmorRectHistoricalDataList& lastArmorRectData, struct ArmorRectHistoricalDataList& nowArmorRectData, size_t i) {
    //填充新数据
    lastArmorRectData.areaChangeRate.back() = nowArmorRectData.areaChangeRate[i];
    lastArmorRectData.lenWidChangeRate.back() = nowArmorRectData.lenWidChangeRate[i];
    lastArmorRectData.xChangeRate.back() = nowArmorRectData.xChangeRate[i];
    lastArmorRectData.yChangeRate.back() = nowArmorRectData.yChangeRate[i];

    nowArmorRectData.areaChangeRateChangeRateVarianceList.push_back(Util::varianceDataCalculate(lastArmorRectData.areaChangeRate));
    nowArmorRectData.lenWidChangeRateVarianceList.push_back(Util::varianceDataCalculate(lastArmorRectData.lenWidChangeRate));
    nowArmorRectData.xChangeRateVarianceList.push_back(Util::varianceDataCalculate(lastArmorRectData.xChangeRate));
    nowArmorRectData.yChangeRateVarianceList.push_back(Util::varianceDataCalculate(lastArmorRectData.yChangeRate));
}

std::vector<ArmorDistinguish::ArmorStruct> ArmorDistinguish::getAttackOrder(std::vector<ArmorStruct>& armorStructs) {
    if (armorStructs.empty()) {
        return vector<ArmorStruct>();
    }
    else if (armorStructs.size() == 1) {
        return armorStructs;
    }
    else {
        vector<bool> orderFlag(3, false);
        //如果工程和其他车都存在，现打其他车
        for (size_t i = 0; i < armorStructs.size(); ++i) {
            if (armorStructs[i].carId == 1 && orderFlag[0] == false) {
                orderFlag[0] = true;
            }
            else if ((armorStructs[i].carId == 3 || armorStructs[i].carId == 4) && orderFlag[1] == false) {
                orderFlag[1] = true;
            }
            else if (armorStructs[i].carId == 2 && orderFlag[2] == false) {
                orderFlag[2] = true;
            }
        }
        //有英雄优先先攻击英雄
        if (orderFlag[0] == true) {
            for (size_t i = 0; i < armorStructs.size(); ++i) {
                if (armorStructs[i].carId == 1)
                    orderArmorStructs.push_back(armorStructs[i]);
            }
        }
        //除了英雄优先攻击步兵
        else if (orderFlag[1] == true) {
            for (size_t i = 0; i < armorStructs.size(); ++i) {
                if (armorStructs[i].carId == 3 || armorStructs[i].carId == 4) {
                    orderArmorStructs.push_back(armorStructs[i]);
                }
            }
        }
        else {
            orderArmorStructs = armorStructs;
        }

        return orderArmorStructs;
    }
}

//小陀螺识别
void ArmorDistinguish::topProcess(float yawAngle) {

    //寻找小陀螺
    if (_topStatusFlag == false) {

        //上一帧丢失装甲板后如果出现sameTargetFlag为false
        if (_losesameFlag == true && _sameTargetFlag == false) {
            _findsameFlag = true;
        }
        //计算1->0->.....->0->1->1周期时间间隔
        else if (_findsameFlag == true && _sameTargetFlag == true && _lastsameTargetFlag == true) {
            _oneCycletime = (double)((cv::getTickCount() - _topTime) / cv::getTickFrequency());
            _losesameFlag = false;
            _findsameFlag = false;
            //std::cout << "oneCycletime:" << _oneCycletime << endl;
        }
        //上次sameTargetFlag为true而现在为false，则认为寻找下一块装甲板
        else if (_losesameFlag == false && _lastsameTargetFlag == true && _sameTargetFlag == false) {
            _topTime = (double)(cv::getTickCount());
            _losesameFlag = true;
        }

        //如果一次1->0->.....->0->1->1周期大于一定范围认为小陀螺旋转1/4个周期
        //周期间隔在0.02秒和0.6秒之间，则认为是正确
        if (_oneCycletime > 0.02 && _oneCycletime < 0.6) {
            _cycleCount++;
            //如果上次角度大于现在角度，则认为是顺时针旋转
            if (_lastAngle > yawAngle) {
                Clockwise++;
            }
            //如果上次角度小于现在角度，则认为是逆时针旋转
            else if (_lastAngle < yawAngle) {
                antiClockwise++;
            }
            _oneCycletime = 0;
            //std::cout << "_cycleCount: " << _cycleCount << endl;
            //std::cout << "Clockwise: " << Clockwise << endl;
            //std::cout << "antiClockwise: " << antiClockwise << endl;
        }
        else if (_oneCycletime != 0) {
            _diffCount++;
        }

        if (Clockwise != 0 || antiClockwise != 0) {
            //如果顺时针和逆时针前后出现都为1
            if (Clockwise == antiClockwise == 1) {
                _diffCount = 4;
                Clockwise = 0;
                antiClockwise = 0;
            }
        }

        //不同计数等于4全部清零
        if (_diffCount == 4) {
            _oneCycletime = 0;
            _cycleCount = 0;
            _diffCount = 0;
        }

        //如果小陀螺旋转大于一个周期认为是小陀螺模式
        if (_cycleCount == 3) {
            if (Clockwise >= 2) {
                _cycleCount = 0;
                _diffCount = 0;
                _oneCycletime = 0;
                _topStatusFlag = true;
                _clockWiseTop = true;
                //std::cout << "clockwiseTop" << endl;
            }
            else if (antiClockwise >= 2) {
                _cycleCount = 0;
                _diffCount = 0;
                _oneCycletime = 0;
                _topStatusFlag = true;
                _antiClockWiseTop = true;
                //std::cout << "anti-clockwiseisTop" << endl;
            }
        }
        else if (_cycleCount > 3 || Clockwise > 3 || antiClockwise > 3) {
            _cycleCount = 0;
            Clockwise = 0;
            antiClockwise = 0;
        }
    }

    //小陀螺模式
    if (_topStatusFlag == true) {

        //两帧之间角度发生大幅度跳变，就把跟踪计数清零
        if (_clockWiseTop == true) {
            if (yawAngle - _lastAngle > 4 && yawAngle - _lastAngle < 12) {
                _trackCount = 0;
                _untrackCount = 0;
            }
        }
        else {
            if (yawAngle - _lastAngle < -4 && yawAngle - _lastAngle > -12) {
                _trackCount = 0;
                _untrackCount = 0;
            }
        }

        //上一帧为丢失而现在一帧为跟踪，就把跟踪计数清零
        if (_sameTargetFlag == true && _lastsameTargetFlag == false) {
            _trackCount = 0;
            _untrackCount = 0;
        }
        //上一帧为跟踪而现在一帧为丢失，就把跟踪计数清零
        else if (_sameTargetFlag == false && _lastsameTargetFlag == true) {
            _trackCount = 0;
            _untrackCount = 0;
        }
        //连续跟踪，增加一次跟踪计数
        else if (_sameTargetFlag == true && _lastsameTargetFlag == true) {
            _trackCount++;
        }
        //连续丢失，增加一次丢失跟踪计数
        else if (_sameTargetFlag == false && _lastsameTargetFlag == false) {
            _untrackCount++;
        }

        //在小陀螺模式，能连续跟随150帧以上，说明已经不是小陀螺
        if (_trackCount > 150) {
            _topStatusFlag = false;
            _trackCount = 0;
            _clockWiseTop = false;
            _antiClockWiseTop = false;
        }
        //在小陀螺模式，能连续丢失150帧以上，说明已经不是小陀螺
        else if (_untrackCount > 150) {
            _topStatusFlag = false;
            _untrackCount = 0;
            _clockWiseTop = false;
            _antiClockWiseTop = false;
        }
    }

    _lastTopStatusFlag = _topStatusFlag;
    _lastsameTargetFlag = _sameTargetFlag;
    _lastAngle = yawAngle;
}

//得到最终识别目标
cv::RotatedRect ArmorDistinguish::getResult() {
    if (armorStructs.empty()) {
        return cv::RotatedRect();
    }
    cv::RotatedRect resultRect_;
#ifdef USE_BP
    //对ID进行攻击序列排序
    //armorStructs = getAttackOrder(armorStructs);
#endif // USE_BP
#ifdef USE_WEIGHT
    _timeBias = (double)((cv::getTickCount() - _lastTime) / cv::getTickFrequency());                //目标捕捉时间偏差
    if (_timeBias > 3.0) {
        _sameTargetFlag = false;
    }                                                                                               //两次目标捕捉时间过长
#ifdef DEBUG_WEIGHT
    static cv::RotatedRect lastRect;
#endif
    //判断是否有历史装甲板数据
    if (_armorRectHistoricalData.area.size() < 1 || !_sameTargetFlag) {
        //无历史装甲板数据是取距离中心最近的装甲板做跟随目标
        //找到深度信息最小的小装甲板序号或大装甲板
        findNearestArmor(armorStructs, resultRect_);
        armorRectListDataReset(_armorRectHistoricalData);                    //重置装甲板历史数据
        armorRectListBaseDataIndex(_armorRectHistoricalData, resultRect_);    //装甲板基本数据装填
        _otherTargetCount = 0;        //跟随错误计数清零
        _sameTargetFlag = true;        //确定跟随目标
    }
    else {
        struct ArmorRectHistoricalDataList armorRectData;                    //有历史装甲板数据权重判断装甲板做跟随目标
        findArmorByWeight(armorStructs, resultRect_, armorRectData);
        if (_maxWeightValue > _armorWeightCalculateCoefficient.sameTargetThreshold && _nowCarID == _lastCarID) {                //权重阈值存取最大权重目标
            _otherTargetCount = 0;
            armorRectListBaseDataIndex(_armorRectHistoricalData, resultRect_);                        //装甲板基本数据装填
            armorRectListBiasDataIndex(_armorRectHistoricalData, armorRectData, maxWeightNum, 0);    //装甲板偏差数据装填
        }
        else {
            _otherTargetCount++;                                                                        //跟随错误计数
            if (_otherTargetCount > OTHER_TARGET_COUNT_LIMIT) {
                _sameTargetFlag = false;
#ifdef DEBUG_WEIGHT
                cout << endl;
                cout << "area:  " << armorRectData.areaChangeRateChangeRateVarianceList[maxWeightNum] * _armorWeightCalculateCoefficient.areaVarianceCoefficient << endl;
                cout << "x:  " << armorRectData.xChangeRateVarianceList[maxWeightNum] * _armorWeightCalculateCoefficient.xVarianceCoefficient << endl;
                cout << "y:  " << armorRectData.yChangeRateVarianceList[maxWeightNum] * _armorWeightCalculateCoefficient.yVarianceCoefficient << endl;
                cout << "lw:  " << armorRectData.lenWidChangeRateVarianceList[maxWeightNum] * _armorWeightCalculateCoefficient.lenWidVarianceCoefficient << endl;
                cout << "error:  " << _maxWeightValue << endl;//确定丢失目标
                //cout << "error" << endl;//确定丢失目标
#endif
            }
        }
    }
#else
    std::vector<float> resultDistance;
    cv::Point2f centerVision((float)(_src.cols / 2), (float)(_src.rows / 2));
    findNearestArmor(realStructs, resultRect_);

#endif  //USE_WEIGHT
    _lastTargetCenter = resultRect_.center;
    //_lastCarID = _nowCarID;
    return resultRect_;
}


//识别过程函数
cv::RotatedRect ArmorDistinguish::process(const cv::Mat& src, EnemyColor enemyColor, CarType carType, bool isReset, DistinguishMode distinguishMode, float yawAngle, bool topRest) {
    if (isReset) {
        _restoreRect = cv::Rect(0, 0, src.cols, src.rows);
    }
    _resultRect = cv::RotatedRect();
    _carType = carType;
    _armorMode = distinguishMode;
#ifdef USE_KCF
    armorTrackerInit(src, enemyColor);
#else
    imagePreprocess(src, enemyColor);
#endif
    //从轮廓中找出类似灯条矩形
    findLightBarContour();
    //std::cout << "possibleRects:" << possibleRects.size() << std::endl;
    //类似灯条矩形过滤器
    lightBarFilter();
    removeWrongArmor();
    //std::cout << armorStructs.size() << std::endl;
#ifdef USE_SHOW
    //不适用调试时可以隐去
    showDistinguishArea(stickerRect);
#endif
    _bigArmorFlag = false;
    armorChooseTarget(enemyColor);
    //std::cout << realStructs.size() << std::endl << std::endl;
    //选择一个最优目标
    _leftLightBar = cv::RotatedRect();
    _rightLightBar = cv::RotatedRect();
    _resultRect = getResult();
    if (_resultRect.size.width > 0) {
        //中间点的坐标再加上restoreRect的坐标
#ifdef USE_KCF
        _isReulstFind = true;
        _isTrackerStart = true;
        if (_isReulstFind) {
            float lightBarWidthL = _leftLightBar.size.width < _leftLightBar.size.height ? _leftLightBar.size.width : _leftLightBar.size.height;
            float lightBarHeightL = _leftLightBar.size.width > _leftLightBar.size.height ? _leftLightBar.size.width : _leftLightBar.size.height;
            float lightBarWidthR = _rightLightBar.size.width < _rightLightBar.size.height ? _rightLightBar.size.width : _rightLightBar.size.height;
            float lightBarHeightR = _rightLightBar.size.width > _rightLightBar.size.height ? _rightLightBar.size.width : _rightLightBar.size.height;

            cv::Point2d recttl = cv::Point2d((double)(_leftLightBar.center.x - 2.0 * lightBarWidthL), (double)(_leftLightBar.center.y - 1.5 * lightBarHeightL));
            cv::Point2d rectbr = cv::Point2d((double)(_rightLightBar.center.x + 2.0 * lightBarWidthR), (double)(_rightLightBar.center.y + 1.5 * lightBarHeightR));
            recttl += cv::Point2d(_searchRect.x, _searchRect.y);
            rectbr += cv::Point2d(_searchRect.x, _searchRect.y);
            _trackerRect = cv::Rect2d(recttl, rectbr);
            _trackerRect &= cv::Rect2d(0, 0, _imgWidth, _imgHeight);
            _tracker = cv::TrackerKCF::create();
            _tracker->init(_src, _trackerRect);
        }
        resultRect.center += cv::Point2f((float)_searchRect.x, (float)_searchRect.y);
#else
        _resultRect.center += cv::Point2f((float)_restoreRect.x, (float)_restoreRect.y);
#endif
        _resultRect.points(_vertices);
        //记录上一刻的检测区域
        _resLast = _resultRect;
        _lost_cnt = 0;
        //记录上一刻的时间
        _lastTime = (double)(cv::getTickCount());

    }
    else {
#ifdef USE_KCF
        _isTrackerStart = false;
        _isReulstFind = false;
        return cv::RotatedRect();
#else
        ++_lost_cnt;
        //丢失目标后逐步扩大搜索范围
        if (carType == NEW_SENTRY_ABOVE || carType == NEW_SENTRY_BELOW || carType == OLD_SENTRY_ABOVE || carType == OLD_SENTRY_BELOW) {
            if (_lost_cnt < 3)
                _resLast.size = cv::Size2f((float)(_resLast.size.width * 3), (float)(_resLast.size.height * 3));
            else if (_lost_cnt==6)
                _resLast.size = cv::Size2f((float)(_resLast.size.width * 1.5), (float)(_resLast.size.height * 1.5));
            else if (_lost_cnt == 9)
                _resLast.size = cv::Size2f((float)(_resLast.size.width * 1.5), (float)(_resLast.size.height * 1.5));
            else if (_lost_cnt == 12)
                _resLast.size = cv::Size2f((float)(_resLast.size.width * 1.5), (float)(_resLast.size.height * 1.5));
            else if (_lost_cnt == 18)
                _resLast.size = cv::Size2f((float)(_resLast.size.width * 1.5), (float)(_resLast.size.height * 1.5));
            else if (_lost_cnt > 60)
                _resLast = cv::RotatedRect();
        }
        else {
            if (_lost_cnt == 8)
                _resLast.size = cv::Size2f((float)(_resLast.size.width * 1.5), (float)(_resLast.size.height * 1.5));
            else if (_lost_cnt == 12)
                _resLast.size = cv::Size2f((float)(_resLast.size.width * 1.5), (float)(_resLast.size.height * 1.5));
            else if (_lost_cnt == 18)
                _resLast.size = cv::Size2f((float)(_resLast.size.width * 1.5), (float)(_resLast.size.height * 1.5));
            else if (_lost_cnt > 60)
                _resLast = cv::RotatedRect();
        }
        _sameTargetFlag = false;
#endif //USE_KCF
    }

    topProcess(yawAngle);                        //识别小陀螺
    _topData.topFlag = _topStatusFlag;

    possibleRects.clear();
    armorStructs.clear();
    allContours.clear();                                //轮廓
    hierarchy.clear();
    orderArmorStructs.clear();
    //std::cout << "_resultRect.center.x:" << _resultRect.center.x << std::endl << std::endl;
    return _resultRect;
}