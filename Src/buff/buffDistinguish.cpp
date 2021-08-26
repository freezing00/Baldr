#include "buffDistinguish.h"
#include <queue>

#define USE_TRAIN                1


BuffDistinguish::BuffDistinguish() {
    _circleCenterSvm = cv::Algorithm::load<cv::ml::SVM>("../trainData/CIRCLE_SVM_DATA.xml");
    _findRectFlag = false;
}

//ͼƬ׼��
void BuffDistinguish::imagePreprocess(const cv::Mat &src, OwnColor ownColor) {
    cv::Point lastResult = _resLast.center;  //��һ�����н��ȡ��һ�η��������
    if (lastResult.x == 0 || lastResult.y == 0) {                        //�����һ�ν����x��yΪ0����������ͼƬ�Ĵ�С������
        _src = src;                                                        //�̳�ԭͼƬ��ַ
        _restoreRect = cv::Rect(0, 0, src.cols, src.rows);
    } else {
        cv::Rect rect = _resLast.boundingRect();                            //���ذ�����ת���ε���С�����������Ρ�
        static float scale = 1.5f;
        int x1 = MAX(int(lastResult.x - (rect.width * scale)), 0);
        int y1 = MAX(int(lastResult.y - (rect.height * scale)), 0);
        cv::Point lu = cv::Point(x1, y1);                                                           //�õ���lu��ROI���Ͻǵ����
        int x2 = MIN(int(lastResult.x + (rect.width * scale)), src.cols - 1);                //������������ͼƬ����
        int y2 = MIN(int(lastResult.y + (rect.height * scale)), src.rows - 1);
        cv::Point rd = cv::Point(x2, y2);                                                           //�õ���rd��ROI���½ǵ��յ�
        _lu = lu;
        _rd = rd;

        _restoreRect = cv::Rect(lu, rd);
        src(_restoreRect).copyTo(_src);
    }
    //}
    _splitSrc.clear();
    split(_src, _splitSrc);                                                                                //����ɫ��ͨ��
    cv::cvtColor(_src, _graySrc, cv::COLOR_BGR2GRAY);                                                                //��ȡ�Ҷ�ͼ
    if (ownColor == OWN_RED) {
        //�ҷ�Ϊ��ɫ
        cv::threshold(_graySrc, _graySrc, _para.grayThreshold_RED, 255,
                      cv::THRESH_BINARY);                            //�Ҷȶ�ֵ��
        cv::subtract(_splitSrc[2], _splitSrc[0],
                     _separationSrc);                                                //����ͨ�����
        cv::threshold(_separationSrc, _separationSrc, _para.separationThreshold_RED, 255,
                      cv::THRESH_BINARY);        //������ֵ��
        cv::subtract(_splitSrc[2], _splitSrc[1], _separationSrcGreen);   //����ͨ�����

        cv::dilate(_separationSrc, _separationSrc,
                   Util::structuringElement3());
        cv::dilate(_separationSrcGreen, _separationSrcGreen, Util::structuringElement3());
        _maxColor = _separationSrc & _graySrc &
                    _separationSrcGreen;                                                                //�߼��������ն�ֵ��ͼ��
        cv::dilate(_maxColor, _maxColor,
                   Util::structuringElement3());
    } else {
        //�ҷ�Ϊ��ɫ
        cv::threshold(_graySrc, _graySrc, _para.grayThreshold_BLUE, 255,
                      cv::THRESH_BINARY);
        //�Ҷȶ�ֵ��
        cv::subtract(_splitSrc[0], _splitSrc[2],
                     _separationSrc);                                                //����ͨ�����
        cv::threshold(_separationSrc, _separationSrc, _para.separationThreshold_BLUE, 255,
                      cv::THRESH_BINARY);        //������ֵ��
        cv::subtract(_splitSrc[0], _splitSrc[1], _separationSrcGreen);
        cv::dilate(_separationSrcGreen, _separationSrcGreen, Util::structuringElement3());
        _maxColor = _separationSrc & _graySrc &
                    _separationSrcGreen;                                                                //�߼��������ն�ֵ��ͼ��
        cv::dilate(_maxColor, _maxColor,
                   Util::structuringElement3());                                                    //��ʴ
    }
}

//�ҳ����⴦������
void BuffDistinguish::findLightBarContour(std::vector<cv::RotatedRect> &rects, std::vector<float> &areas) {
    _suspicion_R_Rects.clear();
    vector<vector<cv::Point2i> > allContours;    //����
    vector<cv::Vec4i> hierarchy;
    cv::findContours(_maxColor, allContours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    vector<vector<cv::Point2i> >::iterator it = allContours.begin();
    int i = 0;
    while (it != allContours.end()) {
        cv::RotatedRect scanRect = minAreaRect(*it);   //�����С����ľ���
        float area = float(cv::contourArea(allContours[i]));
        i++;
        ++it;
        //��һ������
        //rect�ĸ߶ȡ��Ϳ����һ��С�ڰ�Ƶ���С�߶Ⱦ�ֱ����������ѭ��
        float longSide = Util::findExtremumOfSide(scanRect, LONG_SIDE);
        float shortSide = Util::findExtremumOfSide(scanRect, SHORT_SIDE);
        //_suspicion_R_Rects.push_back(scanRect);
        if (longSide < shortSide * 1.2) {
            _suspicion_R_Rects.push_back(scanRect);
            //continue;
        }
        if (scanRect.size.area() <= 2 * _circleCenterRect.size.area()) {
            continue;
        }
        rects.push_back(scanRect);
        areas.push_back(area);
    }
}

//ɸ��������Ƕ����
void BuffDistinguish::findPossibleRects(vector<cv::RotatedRect> rects, vector<float> &areas,
                                        vector<cv::RotatedRect> &cantileverRects, vector<cv::RotatedRect> &armorRects) {
    if (rects.size() <= 0) {
        return;
    }
    float minScale = 1;
    float maxScale = 12;
    _buffAlBeats.clear();
    vector<int> armorNums;
    for (int i = 0; i < rects.size(); i++) {

        armorNums.clear();
        if (Util::pointDistance(rects[i].center, _R_2D_Center) > 12 * _circleCenterLong) {
            continue;
        }
        int armorNum = 0;
        int embeddedNum = 0;
        for (int j = 0; j < rects.size(); j++) {
            if (Util::pointDistance(rects[j].center, _R_2D_Center) > 12 * _circleCenterLong) {
                continue;
            }
            if (j == i) {
                continue;
            }
            //���α�����Ƕ����һ������
            if (Util::doesRectangleContainPoint(rects[i], rects[j]) > 3) {
                //��Ƕ������һ
                embeddedNum++;
                float longSide = Util::findExtremumOfSide(rects[j], LONG_SIDE);
                float shortSide = Util::findExtremumOfSide(rects[j], SHORT_SIDE);
                //����ȱ�����һ����Χ�ڣ�����ż�¼Ϊװ�װ�
                if (longSide / shortSide < 1.9 && longSide / shortSide > 1.5) {
                    armorNums.push_back(j);
                    armorNum = j;
                }
            }
        }
        //����Բ�ľ����ϵɸ��������������װ�װ�
        for (int k = 0; k < armorNums.size(); k++) {
            if (Util::pointDistance(rects[armorNums[k]].center, _R_2D_Center) < 9 * _circleCenterLong) {
                continue;
            }
            armorNum = armorNums[k];
            break;
        }
        if (embeddedNum > 0 && embeddedNum < 5) {  //�޳���Ƕ�ٺ���Ƕ������
            if (Util::pointDistance(rects[i].center, _R_2D_Center) / _circleCenterLong < minScale ||
                Util::pointDistance(rects[i].center, _R_2D_Center) / _circleCenterLong > maxScale) {
                continue;
            }

            BuffAlBeat buffAlBeat;
            cantileverRects.push_back(rects[i]);
            armorRects.push_back(rects[armorNum]);
            buffAlBeat.cantileverRect = rects[i];
            buffAlBeat.cantileverContoursAreas = areas[i];
            buffAlBeat.armorRect = rects[armorNum];
            buffAlBeat.embeddedNum = embeddedNum;
            _buffAlBeats.push_back(buffAlBeat);
        }
    }
    cv::Mat testBuff = _src.clone();
    for (int i = 0; i < _buffAlBeats.size(); i++) {
        Util::drawRect(testBuff, _buffAlBeats[i].cantileverRect);
        Util::drawRect(testBuff, _buffAlBeats[i].armorRect);
    }
}

void BuffDistinguish::distinguishBuffStruct(BuffToBeat &buffToBeat, vector<BuffAlBeat> &buffAlBeat, vector<BuffAlBeat> &alBuffStruct) {

    if (alBuffStruct.size() <= 0) {
        return;
    }
    float minRatio = 100.0;
    int minNumber = 0;
    float area;
    float areaRatio;
    float minCantileverArea;
    cv::Mat testBuff = _src.clone();
    for (int i = 0; i < alBuffStruct.size(); i++) {
        area = alBuffStruct[i].cantileverContoursAreas;
        areaRatio = area / alBuffStruct[i].cantileverRect.size.area();
        if (areaRatio < minRatio) {
            //cout << "area    " << area << endl;
            minRatio = areaRatio;
            minNumber = i;
            //cout << "yes" << areaRatio << endl;
        }
    }

    buffToBeat.armorRect = alBuffStruct[minNumber].armorRect;
    buffToBeat.cantileverRect = alBuffStruct[minNumber].cantileverRect;
    for (int i = 0; i < alBuffStruct.size(); i++) {
        minCantileverArea = alBuffStruct[i].cantileverRect.size.area() * 0.3f;
        if (i == minNumber) {
            continue;
        }
        buffAlBeat.push_back(alBuffStruct[i]);
        //cout << "no" << areaRatio << endl;
    }
}

void BuffDistinguish::makeBuffStruct(vector<cv::RotatedRect> cantileverRects, vector<cv::RotatedRect> armorRects,
                                     BuffToBeat &realBuffToBeat) {
    if (cantileverRects.size() <= 0) {
        return;
    }
    BuffToBeat buffToBeat;
    vector<BuffAlBeat> buffAlBeats;
    vector<BuffAlBeat> alBuff = _buffAlBeats;
    _buffAlBeats.clear();
    distinguishBuffStruct(buffToBeat, buffAlBeats, alBuff);
    realBuffToBeat = buffToBeat;
    _buffAlBeats = buffAlBeats;
}

//Ѱ���������
void BuffDistinguish::findCantilever(std::vector<cv::RotatedRect> &inputRects, BuffToBeat &buffToBeat, vector<float> &inputAreas) {
    _sameTargetFlag = true;
    std::vector<cv::RotatedRect> rectsVector = inputRects;
    std::vector<float> areasVector = inputAreas;
    if (rectsVector.size() < 1) {
        return;
    }
    vector<cv::RotatedRect> cantileverRects;
    vector<cv::RotatedRect> armorRects;
    findPossibleRects(rectsVector, areasVector, cantileverRects, armorRects);
    makeBuffStruct(cantileverRects, armorRects, buffToBeat);
}

//��Բ��
void BuffDistinguish::findCircleCenter(cv::RotatedRect &circleCenterRect) {
    //�����������Ϊ���򷵻�
    _findRectFlag = false;
    vector<cv::RotatedRect> buffRects = _suspicion_R_Rects;
    if (buffRects.size() <= 0) {
        return;
    }

    bool findFlag = false;
    vector<cv::RotatedRect> possibleCircles;
    //ɸѡԲ�ľ���
    for (int i = 0; i < buffRects.size(); i++) {
        if (buffRects[i].size.area() < _paraCircle.minArea) {
            continue;
        }
        //������������������Σ�������
        if ((buffRects[i].size.width / buffRects[i].size.height) > 1.2 ||
            (buffRects[i].size.width / buffRects[i].size.height) < 0.8) {
            continue;
        }
        possibleCircles.push_back(buffRects[i]);
        //���SVM�жϴ���������
        if (!circleCenterSVM(buffRects[i])) {
            continue;
        }
        //�ҳ���Բ�ĺ�ȷ��Բ�ľ��γ��Ϳ�
        circleCenterRect = buffRects[i];
        findFlag = true;
        break;
    }
    if (!findFlag) {
        for (int i = 0; i < possibleCircles.size(); i++) {
            if (fabs(possibleCircles[i].size.area() - _circleCenterArea) < 30) {
                circleCenterRect = possibleCircles[i];
                findFlag = true;
                break;
            }
        }
    }
    if (!findFlag) {
        _findRectFlag = false;
    } else {
        if (circleCenterRect.size.width > circleCenterRect.size.height) {
            _circleCenterLong = circleCenterRect.size.width;
        } else {
            _circleCenterLong = circleCenterRect.size.height;
        }
        _circleCenterArea = circleCenterRect.size.area();
        _R_2D_Center = circleCenterRect.center;
        _findRectFlag = true;
        ++_find_cnt;
    }
    //Util::drawRectShow(_src.clone(), circleCenterRect);
}

//��SVM��Բ��
bool BuffDistinguish::circleCenterSVM(cv::RotatedRect &inputRect) {
    vector<cv::RotatedRect> outputRects;
    float longSide;
    float shortSide;
    cv::Point2f v[4];
    inputRect.points(v);
    if (Util::pointDistance(v[0], v[1]) < Util::pointDistance(v[2], v[1])) {
        longSide = Util::pointDistance(v[2], v[1]);
        shortSide = Util::pointDistance(v[0], v[1]);
    } else {
        longSide = Util::pointDistance(v[0], v[1]);
        shortSide = Util::pointDistance(v[2], v[1]);
    }

    cv::Rect _bounding_roi = inputRect.boundingRect();
    cv::Mat roi_circleCenter;
    if (_bounding_roi.width < _bounding_roi.height) {
        cv::Point center((_bounding_roi.tl().x + _bounding_roi.br().x) / 2,
                         (_bounding_roi.tl().y + _bounding_roi.br().y) / 2);
        cv::Point half(_bounding_roi.height / 2, _bounding_roi.width / 2);
        _bounding_roi = cv::Rect(center - half, center + half);                                    //���¶�λtl��br
    }
    //�����ϳ���һ����Ϊ�˱�֤����任����������ͼ���м��
    _bounding_roi.x -= (int) (shortSide / 2);
    _bounding_roi.width += (int) shortSide;
    _bounding_roi.y -= (int) (shortSide / 2);
    _bounding_roi.height += (int) shortSide;
    if (!Util::makeRectSafe(_bounding_roi, _src.size())) {
        return bool();
    }
    _src(_bounding_roi).copyTo(roi_circleCenter);
    cv::Mat rectify_target = roi_circleCenter.clone();
    transpose(rectify_target, rectify_target);
    cv::Point half((int) (1.0f * longSide) / 2, (int) (1.0f * shortSide / 2));
    cv::Point roiCenter(rectify_target.cols / 2, rectify_target.rows / 2);
    cv::Point addOther(10, 10);
    cv::Rect sampleRoiRect = cv::Rect(roiCenter - half - addOther, roiCenter + half + addOther);
    sampleRoiRect.x = max(sampleRoiRect.x, 0);                        //ȷ�����β�Խ��
    sampleRoiRect.y = max(sampleRoiRect.y, 0);
    if (!Util::makeRectSafe(sampleRoiRect, rectify_target.size())) {
        return bool();
    }
    cv::Mat sample = rectify_target(sampleRoiRect);//���¶�λtl��br
    cv::resize(sample, sample, cv::Size(30, 30), 0, 0);
    _circleSampleData.image = sample;
    cv::cvtColor(sample, sample, cv::COLOR_BGR2GRAY);
    cv::Mat p = sample.reshape(1, 1);
    p.convertTo(p, CV_32FC1);
    normalize(p, p);
    int response = 0;
    response = (int) _circleCenterSvm->predict(p);
    setSampleCaptureState(true);
    if (!response) {
        _circleSampleData.classifyState = false;
        return false;
    }
    _circleSampleData.classifyState = true;
    return true;
}

//ʶ���������ص���������
cv::RotatedRect BuffDistinguish::process(const cv::Mat &src, OwnColor ownColor) {
    _R_2D_Center = cv::Point();
    cv::RotatedRect resultRect;
    BuffToBeat buffToBeat;
    //׼��ͼƬ
    imagePreprocess(src, ownColor);
    //�ҳ���ֵ��ͼ�������
    vector<cv::RotatedRect> possibleRects;
    vector<float> possibleCoutoursArea;
    findLightBarContour(possibleRects, possibleCoutoursArea);
    //ͼ��ȫ��Χ����Բ��
    findCircleCenter(_circleCenterRect);
    //�Ӿ������ҵ�װ�װ������
    findCantilever(possibleRects, buffToBeat, possibleCoutoursArea);

    //    cv::RotatedRect resultRect = cv::RotatedRect();
    resultRect = buffToBeat.armorRect;
    //resultRect = cv::RotatedRect(resultRect.center,cv::Size2f(resultRect.size.width*0.9,resultRect.size.height*0.9),resultRect.angle);
    if (true) {
        //if(resultRect.center.y<_R_2D_Center.y){
        resultRect = cv::RotatedRect(resultRect.center, cv::Size2f(resultRect.size.width * 0.8f, resultRect.size.height * 0.8f), resultRect.angle);
    }
    //Util::drawRectShow(_src.clone(), resultRect);
    //�õ�������Ҫ�����װ�װ�����
    if (resultRect.size.width <= 0) {
        _findRectFlag = false;
    }
    //if (_findRectFlag && _sameTargetFlag) {
    if ((_findRectFlag && _sameTargetFlag) || _find_cnt > 3) {
        //��¼��һ�̵ļ������
        cv::Point2f srcPointBias = cv::Point2f((float) _restoreRect.x, (float) _restoreRect.y);
        resultRect.center += srcPointBias;
        _R_2D_Center += srcPointBias;
        _resLast = cv::RotatedRect(_R_2D_Center, cv::Size2f(_circleCenterLong * 10.0f, _circleCenterLong * 10.0f), 0);
        _lost_cnt = 0;
        //++_find_cnt;
    } else {
        _find_cnt = 0;
        ++_lost_cnt;
        //��ʧĿ���������������Χ
        if (_lost_cnt < 6)
            _resLast.size = cv::Size2f((float) (_resLast.size.width * 1.5), (float) (_resLast.size.height * 1.5));
        if (_lost_cnt == 9)
            _resLast.size = cv::Size2f((float) (_resLast.size.width * 1.5), (float) (_resLast.size.height * 1.5));
        if (_lost_cnt == 12)
            _resLast.size = cv::Size2f((float) (_resLast.size.width * 1.5), (float) (_resLast.size.height * 1.5));
        else if (_lost_cnt > 10)
            _resLast = cv::RotatedRect();
        _findRectFlag = false;

    }
    float longSide = Util::findExtremumOfSide(resultRect, LONG_SIDE);
    float shortSide = Util::findExtremumOfSide(resultRect, SHORT_SIDE);
    // cout<<"scale = "<<resultRect.size.area()/_circleCenterArea<<endl;
    _last_R_2D_Center = _R_2D_Center;
    return resultRect;
}
