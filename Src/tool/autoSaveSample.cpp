#include "autoSaveSample.h"

#define IMG_FORMAT ".bmp"

AutoSaveSample::AutoSaveSample(DecisionLevelPtr& DecisionLevelPtr, SaveSampleMode mode)
    : decisionLevelPtr(DecisionLevelPtr) {
    _saveSampleMode = mode;
    //创建保存目录
    FileOperation::createDirectory(POSITIVE_SAMPLE_PATH);
    FileOperation::createDirectory(NEGATIVE_SAMPLE_PATH);
    FileOperation::createDirectory(AUTO_SAVE_TRUE_SAMPLE_PATH);
    FileOperation::createDirectory(AUTO_SAVE_FALSE_SAMPLE_PATH);

    FileOperation::createDirectory(BUFF_POSITIVE_SAMPLE_PATH);
    FileOperation::createDirectory(BUFF_NEGATIVE_SAMPLE_PATH);
    FileOperation::createDirectory(BUFF_AUTO_SAVE_TRUE_SAMPLE_PATH);
    FileOperation::createDirectory(BUFF_AUTO_SAVE_FALSE_SAMPLE_PATH);

    FileOperation::createDirectory(CIRCLE_POSITIVE_SAMPLE_PATH);
    FileOperation::createDirectory(CIRCLE_NEGATIVE_SAMPLE_PATH);
    FileOperation::createDirectory(CIRCLE_AUTO_SAVE_TRUE_SAMPLE_PATH);
    FileOperation::createDirectory(CIRCLE_AUTO_SAVE_FALSE_SAMPLE_PATH);

    //获得现有的样本容量
    getImgSampleSize();
    saveTime_ = (double)cv::getTickCount();
}

bool AutoSaveSample::saveSamplesReady() {
    double time = ((double)cv::getTickCount() - saveTime_) / cv::getTickFrequency();
    return decisionLevelPtr->getCaptureState() && time > SAVE_INTERVAL_TIME;
}

void AutoSaveSample::getImgSampleSize() {

    _savePositiveNumber = FileOperation::getFileSizeInOrder(_autoSaveTrueSamplePath, IMG_FORMAT);
    _saveNegativeNumber = FileOperation::getFileSizeInOrder(_autoSaveFalseSamplePath, IMG_FORMAT);

    _saveBuffPositiveNumber = FileOperation::getFileSizeInOrder(_buffAutoSaveTrueSamplePath, IMG_FORMAT);
    _saveBuffNegativeNumber = FileOperation::getFileSizeInOrder(_buffAutoSaveFalseSamplePath, IMG_FORMAT);

    _saveCirclePositiveNumber = FileOperation::getFileSizeInOrder(_circleAutoSaveTrueSamplePath, IMG_FORMAT);
    _saveCircleNegativeNumber = FileOperation::getFileSizeInOrder(_circleAutoSaveFalseSamplePath, IMG_FORMAT);
}

void AutoSaveSample::saveSampleByMode(const struct SampleDataStruct& sampleData, DistinguishMode distinguishMode, uchar distinguishPart) {
    if (distinguishMode == 2) {
        if (distinguishPart == 0) {				//悬臂
            if (sampleData.classifyState) {
                FileOperation::saveImg(_buffAutoSaveTrueSamplePath, _saveBuffPositiveNumber, sampleData.image, IMG_FORMAT);
            }
            else {
                FileOperation::saveImg(_buffAutoSaveFalseSamplePath, _saveBuffNegativeNumber, sampleData.image, IMG_FORMAT);
            }
        }
        else {									//圆心
            if (sampleData.classifyState) {
                FileOperation::saveImg(_circleAutoSaveTrueSamplePath, _saveCirclePositiveNumber, sampleData.image, IMG_FORMAT);
            }
            else {
                FileOperation::saveImg(_circleAutoSaveFalseSamplePath, _saveCircleNegativeNumber, sampleData.image, IMG_FORMAT);
            }
        }
    }
    else {
        if (sampleData.classifyState) {
            FileOperation::saveImg(_autoSaveTrueSamplePath, _savePositiveNumber, sampleData.image, IMG_FORMAT);
        }
        else {
            FileOperation::saveImg(_autoSaveFalseSamplePath, _saveNegativeNumber, sampleData.image, IMG_FORMAT);
        }
    }
}

void AutoSaveSample::saveSample() {
    switch (_saveSampleMode) {
    case SAVE_DISABLE: {
    }
                     break;
    case AUTO_SAVE: {
        if (saveSamplesReady()) {
            saveSampleByMode(decisionLevelPtr->getSampleData(), decisionLevelPtr->getDistinguishMode(), decisionLevelPtr->getDistinguishPart());
            saveTime_ = (double)cv::getTickCount();
        }
    }
    default:
        break;
    }
}

