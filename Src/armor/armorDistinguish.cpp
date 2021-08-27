#include "armorDistinguish.h"

//#define SHOW_IMG
//#define USE_SHOW
//#define USE_TRAIN

#ifdef USE_TRAIN
//#define USE_BP
#endif

#ifndef  USE_KCF
#define USE_WEIGHT                //ʹ��Ȩ�ظ���Ŀ��
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
//kcf׷����
void ArmorDistinguish::armorTrackerInit(const cv::Mat& src, EnemyColor enemyColor) {
    cv::Mat img;
    _size = src.size();
    _para.enemyColor = enemyColor;
    _params.detect_thresh = 0.03f;
    if (_isTrackerStart) {
        //׷��ʧ��
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
            //Խ������Ҫ��ȫͼ����
            if ((_trackerRect & cv::Rect2d(0, 0, _imgWidth, _imgHeight)) != _trackerRect) {
                _searchRect = cv::Rect2d(0, 0, _imgWidth, _imgHeight);
            }
            //����������Ҫ��������
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

//ʶ��ǰ��ͼƬ׼��
void ArmorDistinguish::imagePreprocess(const cv::Mat& src, EnemyColor enemyColor) {

    _para.enemyColor = enemyColor;
    const cv::Point& lastResult = _resLast.center;                         //��һ�����н��ȡ��һ�η��������
    if (lastResult.x == 0 || lastResult.y == 0) {                          //�����һ�ν����x��yΪ0����������ͼƬ�Ĵ�С������
        _src = src;                                                        //�̳�ԭͼƬ��ַ
        _restoreRect = cv::Rect(0, 0, src.cols, src.rows);
        _firstEnterFlag = true;
    }
    else {                                                                //����̳���һ�ε�ͼƬ��С
        _firstEnterFlag = false;
        cv::Rect rect = _resLast.boundingRect();                            //���ذ�����ת���ε���С�����������Ρ�
        static float ratioToWidth = 4.8f;
        static float ratioToHeight = 3.0f;
        int x1 = MAX(int(lastResult.x - (rect.width * ratioToWidth)), 0);
        int y1 = MAX(int(lastResult.y - (rect.height * ratioToHeight)), 0);
        cv::Point lu = cv::Point(x1, y1);                                                           //�õ���lu��ROI���Ͻǵ����
        int x2 = MIN(int(lastResult.x + (rect.width * ratioToWidth)), src.cols);                //������������ͼƬ����
        int y2 = MIN(int(lastResult.y + (rect.height * ratioToHeight)), src.rows);
        cv::Point rd = cv::Point(x2, y2);                                                           //�õ���rd��ROI���½ǵ��յ�
        _restoreRect = cv::Rect(lu, rd);
        src(_restoreRect).copyTo(_src);
    }

    //ͼ���ֵ��
    cv::split(_src, splitSrc);                                                               //����ɫ��ͨ��
    cv::cvtColor(_src, _graySrc, cv::COLOR_BGR2GRAY);                                        //��ȡ�Ҷ�ͼ
    cv::threshold(_graySrc, _separationSrcWhite, 240, 255, cv::THRESH_BINARY);
    cv::bitwise_not(_separationSrcWhite, _separationSrcWhite);

    if (enemyColor == ENEMY_RED) {
        //�з�Ϊ��ɫ
        cv::threshold(_graySrc, _graySrc, _para.grayThreshold_RED, 255, cv::THRESH_BINARY);     //�Ҷȶ�ֵ��
        cv::subtract(splitSrc[2], splitSrc[0], _separationSrc);                                 //����ͨ�����
        cv::subtract(splitSrc[2], splitSrc[1], _separationSrcGreen);                             //����ͨ�����
        cv::threshold(_separationSrc, _separationSrc, _para.separationThreshold_RED, 255, cv::THRESH_BINARY);             //������ֵ��
        cv::threshold(_separationSrcGreen, _separationSrcGreen, _para.separationThreshold_GREEN, 255, cv::THRESH_BINARY);//���̶�ֵ��
        cv::dilate(_separationSrc, _separationSrc, Util::structuringElement3());
        cv::dilate(_separationSrcGreen, _separationSrcGreen, Util::structuringElement3());                                        //����

        _maxColor = _separationSrc & _graySrc & _separationSrcGreen & _separationSrcWhite;                                                                //�߼��������ն�ֵ��ͼ��
        cv::dilate(_maxColor, _maxColor, Util::structuringElement3());                                                    //����
        //cv::morphologyEx(_maxColor, _maxColor, cv::MORPH_OPEN, Util::structuringElement3());
    }
    else {
        cv::threshold(splitSrc[2], _purpleSrc, _para.grayThreshold_PURPLE, 255, cv::THRESH_BINARY);                 //��ֹ��ʶ����ɫ����
        cv::bitwise_not(_purpleSrc, _purpleSrc);
        //�з�Ϊ��ɫ
        cv::threshold(_graySrc, _graySrc, _para.grayThreshold_BLUE, 255, cv::THRESH_BINARY);                        //�Ҷȶ�ֵ��
        cv::subtract(splitSrc[0], splitSrc[2], _separationSrc);
        cv::subtract(splitSrc[0], splitSrc[1], _separationSrcGreen);                                                //����ͨ�����
        cv::threshold(_separationSrc, _separationSrc, _para.separationThreshold_BLUE, 255, cv::THRESH_BINARY);
        cv::threshold(_separationSrcGreen, _separationSrcGreen, _para.separationThreshold_GREEN, 255, cv::THRESH_BINARY);//������ֵ��
        cv::dilate(_separationSrc, _separationSrc, Util::structuringElement3());
        cv::dilate(_separationSrcGreen, _separationSrcGreen, Util::structuringElement3());           //����

        _maxColor = _separationSrc & _graySrc & _separationSrcGreen & _separationSrcWhite & _purpleSrc;                                                                //�߼��������ն�ֵ��ͼ��
        cv::dilate(_maxColor, _maxColor, Util::structuringElement3());                                                    //����
        //cv::morphologyEx(_maxColor, _maxColor, cv::MORPH_OPEN, Util::structuringElement3());
    }
}


//�ҵ�����������
void ArmorDistinguish::findLightBarContour() {
    //�ҵ���������
    cv::findContours(_maxColor, allContours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //ֱ��ͨ������ɸѡ  Ȼ���ͨ����ϵ���ת����ɸѡ
    for (int i = 0; i < allContours.size(); ++i) {
        if (hierarchy[i][3] == -1) {
            cv::RotatedRect scanRect = cv::minAreaRect(allContours[i]);                    //�����С����ľ���

            cv::Point2f vertices[4];
            scanRect.points(vertices);

            if (fabs(vertices[1].x - vertices[3].x) > fabs(vertices[1].y - vertices[3].y))
                continue;
            //std::cout << "scanRect.size.area():" << scanRect.size.area() << std::endl;
            if (scanRect.size.area() < _para.minLightBarArea)
                continue;

            //rect�ĸ߶ȡ��Ϳ����һ��С�ڰ�Ƶ���С�߶Ⱦ�ֱ����������ѭ��
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


//�Է��������ɸѡ,�����������ĵ������
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
                //���һ��Ҳ������һ���ĳ��ȷ�Χ��,���ҲҪ��һ����Χ��
                float heightScale = maxLength_1 > maxLength_2 ? (maxLength_1 / maxLength_2) : (maxLength_2 / maxLength_1);
                if (heightScale < 1.4f) {
                    centre_between = sqrtf((float)pow(possibleRects[i].center.x - possibleRects[j].center.x, 2) + (float)pow(possibleRects[i].center.y - possibleRects[j].center.y, 2));
                    //����װ�׾��εĳ����
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

                        //�����������εĽǶȲ����С��һ��ֵ
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

//����װ�װ�����
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

//�������Ϣ��С��װ�װ�;armorType(0��С��1�Ǵ�)
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
            //ѭ���滻�������ͬʱ�滻װ�װ�ṹ��
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

    //ȥ��������Ƕ������װ�װ�
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

    //��װ�װ幫��ͬһ������ȥ������ȴ�Ļ�нǴ��
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

//����Ȩ����װ�װ�
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
    //�ж�Ŀ��װ�װ�����
    _armorType = armorStructs[maxWeightNum].armorType;
    _leftLightBar = armorStructs[0].partLightBars[0];
    _rightLightBar = armorStructs[0].partLightBars[1];
    _nowCarID = armorStructs[maxWeightNum].carId;
}

//ͨ��ml����Ƿ�Ϊװ�װ�
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

            //��ȡװ�װ���ε���С��Ӿ���
            cv::Rect bounding_roi = rects.boundingRect();
            //���ݲ�ͬ�ĳ�������л����δ�С
            if (bounding_roi.width < bounding_roi.height) {
                cv::Point centre((bounding_roi.tl().x + bounding_roi.br().x) / 2, (bounding_roi.tl().y + bounding_roi.br().y) / 2);
                cv::Point half(bounding_roi.height / 2, bounding_roi.width / 2);
                bounding_roi = cv::Rect(centre - half, centre + half);                                    //���¶�λtl��br
            }
            //�����ϳ���һ����Ϊ�˱�֤����任����������ͼ���м��
            bounding_roi.x -= (int)(longSide / 2);
            bounding_roi.width += (int)longSide;
            bounding_roi.y -= (int)(longSide / 2);
            bounding_roi.height += (int)longSide;
            MakeSafeRect(bounding_roi, _src.size());

            cv::Point2f new_center = rects.center - cv::Point2f((float)bounding_roi.x, (float)bounding_roi.y);    //�����µ����ĵ�
            cv::Mat roi_src = _src(bounding_roi);
            cv::Mat rotation = cv::getRotationMatrix2D(new_center, rects.angle, 1);                            //�õ���ת���ͼ
            cv::Mat rectify_target;
            cv::warpAffine(roi_src, rectify_target, rotation, bounding_roi.size());                        //��һ�η���任

            cv::waitKey(1);

            int minInsert = MIN(rectify_target.rows, rectify_target.cols);                                //ѡ��С��һ�ߣ�Ȧһ�������ε�ROI
            cv::Point tl(rectify_target.cols / 2 - minInsert / 4, rectify_target.rows / 2 - minInsert / 4);
            cv::Point br(rectify_target.cols / 2 + minInsert / 4, rectify_target.rows / 2 + minInsert / 4);
            cv::Rect square = cv::Rect(tl, br);
            cv::Mat square_target = rectify_target(square);
            cv::Mat resizeImg;
            cv::resize(square_target, resizeImg, cv::Size(200, 200), 0, 0);                                        //����װ�ش�С
            cv::cvtColor(resizeImg, _bpImg, cv::COLOR_BGR2GRAY);                                                                        //ǰ��վ�հ�����
            cv::threshold(_bpImg, _bpImg, 0, 255, cv::THRESH_OTSU);                                                 //ʶ��2��װ�װ��ֵ��

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
                //std::cout << "����2��װ�װ�" << std::endl;
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
        classId = classNumber.x;    // ���0��45�ȣ�1��װ�װ�
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

//����������HOG������ȡ����Ԥ����
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

//װ�װ�Э����Ȩ�ؼ���
float ArmorDistinguish::armorVarianceWeightCalculate(ArmorRectHistoricalDataList& data, size_t i) {
    return (1 - (data.areaChangeRateChangeRateVarianceList[i] * _armorWeightCalculateCoefficient.areaVarianceCoefficient + data.xChangeRateVarianceList[i] * _armorWeightCalculateCoefficient.xVarianceCoefficient  \
        + data.yChangeRateVarianceList[i] * _armorWeightCalculateCoefficient.yVarianceCoefficient + data.lenWidChangeRateVarianceList[i] * _armorWeightCalculateCoefficient.lenWidVarianceCoefficient));

}

//�ҵ�Ȩ������װ�װ�
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

//��ʷװ�װ���������
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

//��ʷװ�װ��������װ��
void ArmorDistinguish::armorRectListBaseDataIndex(struct ArmorRectHistoricalDataList& armorRectData, cv::RotatedRect& rect) {
    cv::Point2f resultCenter = cv::Point2f(rect.center.x + _restoreRect.x, rect.center.y + _restoreRect.y);
    if (armorRectData.area.size() < FIFO_LEN) {
        //���������
        armorRectData.area.push_back(rect.size.height * rect.size.width);
        armorRectData.lengthWidthRatio.push_back(Util::getRectLengthWidthRatio(rect));
        armorRectData.x.push_back(resultCenter.x);
        armorRectData.y.push_back(resultCenter.y);
    }
    else {
        //������һ������
        armorRectData.area.erase(std::begin(armorRectData.area));
        armorRectData.lengthWidthRatio.erase(std::begin(armorRectData.lengthWidthRatio));
        armorRectData.x.erase(std::begin(armorRectData.x));
        armorRectData.y.erase(std::begin(armorRectData.y));
        //���������
        armorRectData.area.push_back(rect.size.height * rect.size.width);
        armorRectData.lengthWidthRatio.push_back(Util::getRectLengthWidthRatio(rect));
        armorRectData.x.push_back(resultCenter.x);
        armorRectData.y.push_back(resultCenter.y);
    }
}

//��ʷװ�װ�ƫ������װ��
void ArmorDistinguish::armorRectListBiasDataIndex(struct ArmorRectHistoricalDataList& lastArmorRectData, struct ArmorRectHistoricalDataList& nowArmorRectData, uint8_t i, uint8_t mode) {

    if (lastArmorRectData.areaChangeRate.size() < FIFO_LEN) {
        //���������
        lastArmorRectData.areaChangeRate.push_back(nowArmorRectData.areaChangeRate[i]);
        lastArmorRectData.areaChangeRateChangeRate.push_back(nowArmorRectData.areaChangeRateChangeRate[i]);
        lastArmorRectData.lenWidChangeRate.push_back(nowArmorRectData.lenWidChangeRate[i]);
        lastArmorRectData.xChangeRate.push_back(nowArmorRectData.xChangeRate[i]);
        lastArmorRectData.yChangeRate.push_back(nowArmorRectData.yChangeRate[i]);

    }
    else {
        //������һ������
        lastArmorRectData.areaChangeRate.erase(std::begin(lastArmorRectData.areaChangeRate));
        lastArmorRectData.areaChangeRateChangeRate.erase(std::begin(lastArmorRectData.areaChangeRateChangeRate));
        lastArmorRectData.lenWidChangeRate.erase(std::begin(lastArmorRectData.lenWidChangeRate));
        lastArmorRectData.xChangeRate.erase(std::begin(lastArmorRectData.xChangeRate));
        lastArmorRectData.yChangeRate.erase(std::begin(lastArmorRectData.yChangeRate));
        //���������
        lastArmorRectData.areaChangeRate.push_back(nowArmorRectData.areaChangeRate[i]);
        lastArmorRectData.areaChangeRateChangeRate.push_back(nowArmorRectData.areaChangeRateChangeRate[i]);
        lastArmorRectData.lenWidChangeRate.push_back(nowArmorRectData.lenWidChangeRate[i]);
        lastArmorRectData.xChangeRate.push_back(nowArmorRectData.xChangeRate[i]);
        lastArmorRectData.yChangeRate.push_back(nowArmorRectData.yChangeRate[i]);
    }
}

//װ�װ巽������װ��
void ArmorDistinguish::armorRectListVarianceDataCalculate(struct ArmorRectHistoricalDataList& lastArmorRectData, struct ArmorRectHistoricalDataList& nowArmorRectData, size_t i) {
    //���������
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
        //������̺������������ڣ��ִ�������
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
        //��Ӣ�������ȹ���Ӣ��
        if (orderFlag[0] == true) {
            for (size_t i = 0; i < armorStructs.size(); ++i) {
                if (armorStructs[i].carId == 1)
                    orderArmorStructs.push_back(armorStructs[i]);
            }
        }
        //����Ӣ�����ȹ�������
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

//�õ�����ʶ��Ŀ��
cv::RotatedRect ArmorDistinguish::getResult() {
    if (armorStructs.empty()) {
        return cv::RotatedRect();
    }
    cv::RotatedRect resultRect_;
#ifdef USE_BP
    //��ID���й�����������
    //armorStructs = getAttackOrder(armorStructs);
#endif // USE_BP
#ifdef USE_WEIGHT
    _timeBias = (double)((cv::getTickCount() - _lastTime) / cv::getTickFrequency());                //Ŀ�겶׽ʱ��ƫ��
    if (_timeBias > 3.0) {
        _sameTargetFlag = false;
    }                                                                                               //����Ŀ�겶׽ʱ�����
#ifdef DEBUG_WEIGHT
    static cv::RotatedRect lastRect;
#endif
    //�ж��Ƿ�����ʷװ�װ�����
    if (_armorRectHistoricalData.area.size() < 1 || !_sameTargetFlag) {
        //����ʷװ�װ�������ȡ�������������װ�װ�������Ŀ��
        //�ҵ������Ϣ��С��Сװ�װ���Ż��װ�װ�
        findNearestArmor(armorStructs, resultRect_);
        armorRectListDataReset(_armorRectHistoricalData);                    //����װ�װ���ʷ����
        armorRectListBaseDataIndex(_armorRectHistoricalData, resultRect_);    //װ�װ��������װ��
        _otherTargetCount = 0;        //��������������
        _sameTargetFlag = true;        //ȷ������Ŀ��
    }
    else {
        struct ArmorRectHistoricalDataList armorRectData;                    //����ʷװ�װ�����Ȩ���ж�װ�װ�������Ŀ��
        findArmorByWeight(armorStructs, resultRect_, armorRectData);
        if (_maxWeightValue > _armorWeightCalculateCoefficient.sameTargetThreshold && _nowCarID == _lastCarID) {                //Ȩ����ֵ��ȡ���Ȩ��Ŀ��
            _otherTargetCount = 0;
            armorRectListBaseDataIndex(_armorRectHistoricalData, resultRect_);                        //װ�װ��������װ��
            armorRectListBiasDataIndex(_armorRectHistoricalData, armorRectData, maxWeightNum, 0);    //װ�װ�ƫ������װ��
        }
        else {
            _otherTargetCount++;                                                                        //����������
            if (_otherTargetCount > OTHER_TARGET_COUNT_LIMIT) {
                _sameTargetFlag = false;
#ifdef DEBUG_WEIGHT
                cout << endl;
                cout << "area:  " << armorRectData.areaChangeRateChangeRateVarianceList[maxWeightNum] * _armorWeightCalculateCoefficient.areaVarianceCoefficient << endl;
                cout << "x:  " << armorRectData.xChangeRateVarianceList[maxWeightNum] * _armorWeightCalculateCoefficient.xVarianceCoefficient << endl;
                cout << "y:  " << armorRectData.yChangeRateVarianceList[maxWeightNum] * _armorWeightCalculateCoefficient.yVarianceCoefficient << endl;
                cout << "lw:  " << armorRectData.lenWidChangeRateVarianceList[maxWeightNum] * _armorWeightCalculateCoefficient.lenWidVarianceCoefficient << endl;
                cout << "error:  " << _maxWeightValue << endl;//ȷ����ʧĿ��
                //cout << "error" << endl;//ȷ����ʧĿ��
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


//ʶ����̺���
cv::RotatedRect ArmorDistinguish::process(const cv::Mat& src, EnemyColor enemyColor, CarType carType, bool isReset, DistinguishMode distinguishMode) {
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
    //���������ҳ����Ƶ�������
    findLightBarContour();
    //std::cout << "possibleRects:" << possibleRects.size() << std::endl;
    //���Ƶ������ι�����
    lightBarFilter();
    removeWrongArmor();
    //std::cout << armorStructs.size() << std::endl;
#ifdef USE_SHOW
    //�����õ���ʱ������ȥ
    showDistinguishArea(stickerRect);
#endif
    _bigArmorFlag = false;
    armorChooseTarget(enemyColor);
    //std::cout << realStructs.size() << std::endl << std::endl;
    //ѡ��һ������Ŀ��
    _leftLightBar = cv::RotatedRect();
    _rightLightBar = cv::RotatedRect();
    _resultRect = getResult();
    if (_resultRect.size.width > 0) {
        //�м��������ټ���restoreRect������
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
        //��¼��һ�̵ļ������
        _resLast = _resultRect;
        _lost_cnt = 0;
        //��¼��һ�̵�ʱ��
        _lastTime = (double)(cv::getTickCount());

    }
    else {
#ifdef USE_KCF
        _isTrackerStart = false;
        _isReulstFind = false;
        return cv::RotatedRect();
#else
        ++_lost_cnt;
        //��ʧĿ���������������Χ
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
    possibleRects.clear();
    armorStructs.clear();
    allContours.clear();                                //����
    hierarchy.clear();
    orderArmorStructs.clear();
    //std::cout << "_resultRect.center.x:" << _resultRect.center.x << std::endl << std::endl;
    return _resultRect;
}