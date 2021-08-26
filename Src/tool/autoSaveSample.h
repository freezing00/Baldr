#ifndef _AUTO_SAVE_SAMPLE_H_
#define _AUTO_SAVE_SAMPLE_H_

#include "Infra/Thread.h"
#include "decisionLevel/decisionLevel.h"
#include "string"

#define POSITIVE_SAMPLE_PATH            "../trainData/armor/positive/"
#define NEGATIVE_SAMPLE_PATH            "../trainData/armor/negative/"
#define AUTO_SAVE_TRUE_SAMPLE_PATH      "../trainData/armor/autoSaveSample/true/"
#define AUTO_SAVE_FALSE_SAMPLE_PATH     "../trainData/armor/autoSaveSample/false/"

#define BUFF_POSITIVE_SAMPLE_PATH   "../trainData/buff/positive/"
#define BUFF_NEGATIVE_SAMPLE_PATH   "../trainData/buff/negative/"
#define BUFF_AUTO_SAVE_TRUE_SAMPLE_PATH  "../trainData/buff/autoSaveSample/true/"
#define BUFF_AUTO_SAVE_FALSE_SAMPLE_PATH  "../trainData/buff/autoSaveSample/false/"

#define CIRCLE_POSITIVE_SAMPLE_PATH   "../trainData/circle/positive/"
#define CIRCLE_NEGATIVE_SAMPLE_PATH   "../trainData/circle/negative/"
#define CIRCLE_AUTO_SAVE_TRUE_SAMPLE_PATH  "../trainData/circle/autoSaveSample/true/"
#define CIRCLE_AUTO_SAVE_FALSE_SAMPLE_PATH  "../trainData/circle/autoSaveSample/false/"

#define SAVE_INTERVAL_TIME  1.0f


class AutoSaveSample {
public:
    AutoSaveSample(DecisionLevelPtr &DecisionLevelPtr, SaveSampleMode mode = SAVE_DISABLE);

    /**
     * ���������ⲿ����
     */
    void saveSample();

private:

    double saveTime_;

    SaveSampleMode _saveSampleMode = SAVE_DISABLE;

    DecisionLevelPtr decisionLevelPtr;

    uint32_t _savePositiveNumber = 0;
    uint32_t _saveNegativeNumber = 0;

    uint32_t _saveBuffPositiveNumber = 0;
    uint32_t _saveBuffNegativeNumber = 0;

    uint32_t _saveCirclePositiveNumber = 0;
    uint32_t _saveCircleNegativeNumber = 0;

    string _autoSaveTrueSamplePath = AUTO_SAVE_TRUE_SAMPLE_PATH;
    string _autoSaveFalseSamplePath = AUTO_SAVE_FALSE_SAMPLE_PATH;

    string _buffAutoSaveTrueSamplePath = BUFF_AUTO_SAVE_TRUE_SAMPLE_PATH;
    string _buffAutoSaveFalseSamplePath = BUFF_AUTO_SAVE_FALSE_SAMPLE_PATH;

    string _circleAutoSaveTrueSamplePath = CIRCLE_AUTO_SAVE_TRUE_SAMPLE_PATH;
    string _circleAutoSaveFalseSamplePath = CIRCLE_AUTO_SAVE_FALSE_SAMPLE_PATH;

    /**
     * ��ȡ���е���������
     */
    void getImgSampleSize();

    /**
     * �Ӿ����̻߳�ȡ�Ƿ�׼���ñ���������������������ʱ����Ϊ SAVE_INTERVAL_TIME ������
     * @return flag
     */
    bool saveSamplesReady();

    /**
     * ����ʶ��ģʽ��������ǩ�Զ�������������
     * @param sampleData ��������
     * @param distinguishMode ʶ��ģʽ
     */
    void saveSampleByMode(const struct SampleDataStruct &sampleData, DistinguishMode distinguishMode, uchar distinguishPart);
};

#endif
