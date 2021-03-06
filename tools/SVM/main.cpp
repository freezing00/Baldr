#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <io.h>


//#define USE_RGB


#define POSITIVE_TRAIN_DATA_PATH "D:\\RM\\sample\\saveImg\\true"
#define NEGATIVE_TRAIN_DATA_PATH "D:\\RM\\sample\\saveImg\\false"
#define TEST_DATA_PATH "D:/Robomaster/2020/SVM/样本/test"
//#define TEST_DATA_PATH "../../../TrainData/armor/negative"
#define RESIZE_SIZE_LENGTH 90
#define RESIZE_SIZE_WIDTH 30
#define SAVE_NAME_RGB "BUFF_SVM_DATA_RGB.xml"
#define SAVE_NAME "BUFF_SVM_DATA.xml"


using namespace std;

/**
 * 获得图片名称列表
 * @param path
 * @param files
 */
void getFiles(const string &path, vector<string> &files) {
    long long hFile = 0;
    struct _finddata_t fileInfo;
    string p;
    if ((hFile = _findfirst(p.assign(path).append("/*").c_str(), &fileInfo)) != -1) {
        do {
            if ((fileInfo.attrib & _A_SUBDIR)) {
                if (strcmp(fileInfo.name, ".") != 0 && strcmp(fileInfo.name, "..") != 0)
                    getFiles(p.assign(path).append("/").append(fileInfo.name), files);
            } else {
                files.push_back(p.assign(path).append("/").append(fileInfo.name));
            }
        } while (_findnext(hFile, &fileInfo) == 0);

        _findclose(hFile);
    }
}

/**
 * 获得训练图片
 * @param trainingImages
 * @param trainingLabels
 */
void getTrain(cv::Mat &trainingImages, vector<int> &trainingLabels) {

    string filePathNegative = NEGATIVE_TRAIN_DATA_PATH;
    vector<string> filesNegative;
    getFiles(filePathNegative, filesNegative);
    size_t numberNegative = filesNegative.size();
    for (int i = 0; i < numberNegative; i++) {
        cv::Mat SrcImage = cv::imread(filesNegative[i].c_str());
        resize(SrcImage, SrcImage, cv::Size(RESIZE_SIZE_WIDTH, RESIZE_SIZE_WIDTH));
#ifndef USE_RGB
        cvtColor(SrcImage, SrcImage, cv::COLOR_BGR2GRAY);
#endif
        SrcImage = SrcImage.reshape(1, 1);
        SrcImage.convertTo(SrcImage, CV_32FC1);
        normalize(SrcImage, SrcImage);
        trainingImages.push_back(SrcImage);
        trainingLabels.push_back(0);
    }
    cout << "负样本数量:" << numberNegative << endl;

    string filePathPositive = POSITIVE_TRAIN_DATA_PATH;
    vector<string> filesPositive;
    getFiles(filePathPositive, filesPositive);
    size_t numberPositive = filesPositive.size();
    for (int i = 0; i < numberPositive; i++) {
        cv::Mat SrcImage = cv::imread(filesPositive[i].c_str());
        resize(SrcImage, SrcImage, cv::Size(RESIZE_SIZE_WIDTH, RESIZE_SIZE_WIDTH));
#ifndef USE_RGB
        cvtColor(SrcImage, SrcImage, cv::COLOR_BGR2GRAY);
#endif
        SrcImage = SrcImage.reshape(1, 1);
        SrcImage.convertTo(SrcImage, CV_32FC1);
        normalize(SrcImage, SrcImage);
        trainingImages.push_back(SrcImage);
        trainingLabels.push_back(1);
    }
    cout << "正样本数量:" << numberPositive << endl;
}

/**
 * 测试模型
 * @param filePath
 */
void testModel(const string &filePath) {
#ifdef USE_RGB
    cv::Ptr<cv::ml::SVM> svm = cv::Algorithm::load<cv::ml::SVM>(SAVE_NAME_RGB);
#else
    cv::Ptr<cv::ml::SVM> svm = cv::Algorithm::load<cv::ml::SVM>(SAVE_NAME);
#endif
    cout << "网络模型参数：" << endl;
    cout << "Degree = " << svm->getDegree() << endl;
    cout << "Gamma = " << svm->getGamma() << endl;
    cout << "Coef0 = " << svm->getCoef0() << endl;
    cout << "C = " << svm->getC() << endl;
    cout << "Nu = " << svm->getNu() << endl;
    cout << "P = " << svm->getP() << endl;

    vector<string> files;
    getFiles(filePath, files);

    size_t number = files.size();
    int positiveNumber = 0, negativeNumber = 0;
    double totalTime = 0;
    for (int i = 0; i < number; i++) {
        double t = (double) cv::getTickCount();
        cv::Mat SrcImage = cv::imread(files[i].c_str());
        resize(SrcImage, SrcImage, cv::Size(RESIZE_SIZE_WIDTH, RESIZE_SIZE_WIDTH));
        cvtColor(SrcImage, SrcImage, cv::COLOR_BGR2GRAY);
        cv::Mat p = SrcImage.reshape(1, 1);
        p.convertTo(p, CV_32FC1);
        normalize(p, p);
        int response = (int) svm->predict(p);
        double time = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
        if (response) {
            positiveNumber++;
        } else {
            negativeNumber++;
        }
        totalTime += time;
//        cout << "预测结果：" << response << "  用时：" << fixed << setprecision(8) << time << "秒" << endl;
    }
    double aveTime = totalTime / number;
    cout << "测试图片数量： " << number << endl;
    cout << "平均检测时间：" << aveTime << " 秒 " << endl;
    cout << "判断为1的个数：" << positiveNumber << endl;
    cout << "判断为0的个数：" << negativeNumber << endl;
}

int main() {

    cv::Mat classes;
    cv::Mat trainingData;
    cv::Mat trainingImages;
    vector<int> trainingLabels;

    getTrain(trainingImages, trainingLabels);
    cv::Mat(trainingImages).copyTo(trainingData);
    trainingData.convertTo(trainingData, CV_32FC1);
    cv::Mat(trainingLabels).copyTo(classes);

    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    svm->setType(cv::ml::SVM::Types::C_SVC);
    svm->setKernel(cv::ml::SVM::KernelTypes::RBF);
    svm->setTermCriteria(cv::TermCriteria(CV_TERMCRIT_EPS, 1000, FLT_EPSILON));
    svm->setDegree(0);
    svm->setGamma(1e-05);
    svm->setCoef0(0);
    svm->setC(1);
    svm->setNu(0);
    svm->setP(0);//损失函数p值


    cout << "正在疯狂学习" << endl;
    svm->trainAuto(trainingData, cv::ml::SampleTypes::ROW_SAMPLE, classes);
#ifdef USE_RGB
    svm->save(SAVE_NAME_RGB);
#else
    svm->save(SAVE_NAME);
#endif

    cout << "学完了" << endl;

   // cout << "开始测试模型" << endl;
   // testModel(TEST_DATA_PATH);

    return 0;
}

