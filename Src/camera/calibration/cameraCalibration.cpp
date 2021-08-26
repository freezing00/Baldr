#include "cameraCalibration.h"

bool CCalibration::readPatternImg() {
    uint32_t imgCount = 0;
    FileOperation::getImgFileInOrder(patternImgPath, imgCount, "jpg", patternImgList, 0);
    if (imgCount == 0)return false;
    this->imgNum = imgCount;
    imgHeight = patternImgList[0].rows;
    imgWidth = patternImgList[0].cols;
    return true;
}

bool CCalibration::writeParams() {
    camK.convertTo(camK, CV_32FC1);
    camDiscoeff.convertTo(camDiscoeff, CV_32FC1);

    cv::FileStorage fs(calibResultPath + caliCameraDataName, cv::FileStorage::WRITE);
    fs << "IntrinsicParameters_fx" << camK.at<float>(0, 0);
    fs << "IntrinsicParameters_fy" << camK.at<float>(1, 1);
    fs << "IntrinsicParameters_u0" << camK.at<float>(0, 2);
    fs << "IntrinsicParameters_v0" << camK.at<float>(1, 2);
    fs << "DistortionFactor_k1" << camDiscoeff.at<float>(0, 0);
    fs << "DistortionFactor_k2" << camDiscoeff.at<float>(0, 1);
    fs << "DistortionFactor_p1" << camDiscoeff.at<float>(0, 2);
    fs << "DistortionFactor_p2" << camDiscoeff.at<float>(0, 3);
    fs << "DistortionFactor_k3" << camDiscoeff.at<float>(0, 4);
    fs.release();
    LOG::debug("IntrinsicParameters_fx=" + std::to_string(camK.at<float>(0, 0)));
    LOG::debug("IntrinsicParameters_fy=" + std::to_string(camK.at<float>(1, 1)));
    LOG::debug("IntrinsicParameters_u0=" + std::to_string(camK.at<float>(0, 2)));
    LOG::debug("IntrinsicParameters_v0=" + std::to_string(camK.at<float>(1, 2)));
    LOG::debug("DistortionFactor_k1=" + std::to_string(camDiscoeff.at<float>(0, 0)));
    LOG::debug("DistortionFactor_k2=" + std::to_string(camDiscoeff.at<float>(0, 1)));
    LOG::debug("DistortionFactor_p1=" + std::to_string(camDiscoeff.at<float>(0, 2)));
    LOG::debug("DistortionFactor_p2=" + std::to_string(camDiscoeff.at<float>(0, 3)));
    LOG::debug("DistortionFactor_k3=" + std::to_string(camDiscoeff.at<float>(0, 4)));
    return true;
}

//通过计算三个相邻角点构成的两个向量之间的夹角判断角点连接性
bool CCalibration::testCorners(std::vector<cv::Point2f> &corners, int patternWidth, int patternHeight) {
    if (corners.size() != (size_t) patternWidth * patternHeight) {
        return false;
    }
    double dx1, dx2, dy1, dy2;
    double cosVal;
    for (int i = 0; i < patternHeight; ++i) {
        for (int j = 0; j < patternWidth - 2; ++j) {
            dx1 = corners[i * patternWidth + j + 1].x - corners[i * patternWidth + j].x;
            dy1 = corners[i * patternWidth + j + 1].y - corners[i * patternWidth + j].y;
            dx2 = corners[i * patternWidth + j + 2].x - corners[i * patternWidth + j + 1].x;
            dy2 = corners[i * patternWidth + j + 2].y - corners[i * patternWidth + j + 1].y;
            cosVal = (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
            if (fabs(cosVal) < 0.993) {
                return false;
            }
        }
    }

    for (int i = 0; i < patternHeight - 2; ++i) {
        for (int j = 0; j < patternWidth; ++j) {
            dx1 = corners[(i + 1) * patternWidth + j].x - corners[i * patternWidth + j].x;
            dy1 = corners[(i + 1) * patternWidth + j].y - corners[i * patternWidth + j].y;
            dx2 = corners[(i + 2) * patternWidth + j].x - corners[(i + 1) * patternWidth + j].x;
            dy2 = corners[(i + 2) * patternWidth + j].y - corners[(i + 1) * patternWidth + j].y;
            cosVal = (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
            if (fabs(cosVal) < 0.993) {
                return false;
            }
        }
    }
    return true;
}

//初始化角点的三维坐标
void CCalibration::init3DPoints(const cv::Size &boardSize, const cv::Size &squareSize, std::vector<cv::Point3f> &singlePatternPoint) {
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            cv::Point3f tempPoint;//单个角点的三维坐标
            tempPoint.x = float(i * squareSize.width);
            tempPoint.y = float(j * squareSize.height);
            tempPoint.z = 0;
            singlePatternPoint.push_back(tempPoint);
        }
    }
}

void CCalibration::calibProcess() {
    //***************摄像机标定****************//
    double time0 = (double) cv::getTickCount();
    std::vector<cv::Point2f> corners;//存储一幅棋盘图中的所有角点二维坐标
    std::vector<std::vector<cv::Point2f>> cornersSeq;//存储所有棋盘图角点的二维坐标
    std::vector<cv::Mat> image_Seq;//存储所有棋盘图
    int successImgNum = 0;
    int count = 0;
    LOG::info("********Start extracting the corners! ********");
    cv::Mat image, scaleImg;
    for (int i = 0; i < imgNum; i++) {
        LOG::info("Image#" + std::to_string(i) + ".......");
        image = patternImgList[i].clone();
        //降采样原图,加快角点提取速度
        cv::resize(image, scaleImg, cv::Size(), scale, scale, cv::INTER_LINEAR);
        /**********************提取角点*************************/
        bool patternFound = findChessboardCorners(scaleImg, boardSize,
                                                  corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        if (!patternFound) {
            LOG::error("Can not find chess board corners!");
            continue;
        } else {
            //上采样corner
            for (size_t num = 0; num < corners.size(); num++) {
                cv::Point2f tempPoint = corners[num];
                corners[num] = cv::Point2f(tempPoint.x / scale, tempPoint.y / scale);
            }

            /************************亚像素精确化******************************/
            cv::cornerSubPix(image, corners, cv::Size(15, 15), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.1));
            if (testCorners(corners, boardSize.width, boardSize.height)) {
                /************************绘制检测到的角点并显示******************************/
                cv::Mat cornerImg = image.clone();
                cv::cvtColor(cornerImg, cornerImg, cv::COLOR_GRAY2BGR);
                for (size_t j = 0; j < corners.size(); j++) {
                    cv::circle(cornerImg, corners[j], 20, cv::Scalar(0, 0, 255), 2, 8, 0);
                }
                cv::namedWindow("CirclePattern", 0);
                cv::imshow("CirclePattern", cornerImg);
                cv::waitKey(1);

                count += (int) corners.size();//所有棋盘图中的角点个数
                successImgNum++;//成功提取角点的棋盘图个数
                cornersSeq.push_back(corners);
                image_Seq.push_back(image);
            }
        }
    }
    LOG::info("******* Corner extraction complete! ******");

    /**************************摄像机标定******************************/
    cv::Size squre_size = cv::Size(30, 30);//棋盘格尺寸
    std::vector<std::vector<cv::Point3f>> object_points;//所有棋盘图像的角点三维坐标
    std::vector<int> pointCounts;
    //初始化单幅靶标图片的三维点
    init3DPoints(boardSize, squre_size, singlePatternPoints);
    //初始化标定板上的三维坐标
    for (int n = 0; n < successImgNum; n++) {
        object_points.push_back(singlePatternPoints);
        pointCounts.push_back(boardSize.width * boardSize.height);
    }

    /***开始标定***/
    LOG::info("***** To start calibration! ******");
    cv::Size imgSize = cv::Size(imgWidth, imgHeight);
    std::vector<cv::Vec3d> rotation;//旋转向量
    std::vector<cv::Vec3d> translation;//平移向量

    int flags = 0;
    cv::calibrateCamera(object_points, cornersSeq, imgSize, camK, camDiscoeff, rotation, translation, flags);

    LOG::info("*********************** Calibration completed! **************************");
    LOG::info("Calibration time :" + std::to_string(((double) cv::getTickCount() - time0) / cv::getTickFrequency()) + "s");
    LOG::info("Calibration Successful Number:" + std::to_string(successImgNum));
    //评价
    std::vector<int> outLierIndex;
    evaluateCalibrationResult(object_points, cornersSeq, rotation, translation,
                              camK, camDiscoeff, successImgNum, outLierIndex, (int) errThresh);
    //删除误差大的角点图
    std::vector<std::vector<cv::Point2f>> newCornersSeq;
    successImgNum = 0;
    for (size_t i = 0; i < cornersSeq.size(); i++) {
        if (outLierIndex[i] == 0) {
            newCornersSeq.push_back(cornersSeq[i]);
            successImgNum++;
        }
    }
    std::vector<std::vector<cv::Point3f>> newObjectPoints;
    for (int n = 0; n < successImgNum; n++) {
        newObjectPoints.push_back(singlePatternPoints);
    }
    //重新标定
    cv::calibrateCamera(object_points, cornersSeq, imgSize, camK, camDiscoeff,
                        rotation, translation, flags);
    //重新计算重投影误差
    outLierIndex.clear();
    evaluateCalibrationResult(object_points, cornersSeq, rotation, translation,
                              camK, camDiscoeff, successImgNum, outLierIndex, (int) errThresh);
#ifdef DEBUG
    //通过畸变校正效果查看摄像机标定效果
    cv::Mat R = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat mapx, mapy, newCamK, undistortImg, showImg;
    cv::initUndistortRectifyMap(camK, camDiscoeff, R, camK, imgSize, CV_32FC1, mapx, mapy);
    cv::remap(image_Seq[0], undistortImg, mapx, mapy, CV_INTER_LINEAR);
    cv::resize(undistortImg, showImg, cv::Size(), 0.25, 0.25, CV_INTER_LINEAR);
    string winName = "undistortImg";
    cv::namedWindow(winName, 1);
    cv::imshow(winName, showImg);
    cv::waitKey(0);
#endif
}

//估计重投影误差，并排除误差大于设定阈值的靶标图片
int CCalibration::evaluateCalibrationResult(std::vector<std::vector<cv::Point3f>> objectPoints, std::vector<std::vector<cv::Point2f>> cornerSquare, std::vector<cv::Vec3d> _rvec,
                                            std::vector<cv::Vec3d> _tvec, cv::Mat _K, cv::Mat _D, int count, std::vector<int> &outLierIndex, int errThresh) {
    std::string evaluatPath = calibResultPath + caliBiasDataName;
    std::ofstream fout(evaluatPath);

    std::vector<cv::Point2f> proImgPoints;
    for (int i = 0; i < count; i++) {
        float maxValue = -1;
        std::vector<cv::Point3f> tempPointSet = objectPoints[i];
        cv::projectPoints(tempPointSet, _rvec[i], _tvec[i], _K, _D, proImgPoints);

        std::vector<cv::Point2f> tempImgPoint = cornerSquare[i];
        cv::Mat tempImgPointMat = cv::Mat(1, (int) tempImgPoint.size(), CV_32FC2);
        cv::Mat proImgPointsMat = cv::Mat(1, (int) proImgPoints.size(), CV_32FC2);
        for (int j = 0; j != (int) tempImgPoint.size(); j++) {
            proImgPointsMat.at<cv::Vec2f>(0, j) = cv::Vec2f(proImgPoints[j].x, proImgPoints[j].y);
            tempImgPointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImgPoint[j].x, tempImgPoint[j].y);
            float dx = proImgPoints[j].x - tempImgPoint[j].x;
            float dy = proImgPoints[j].y - tempImgPoint[j].y;
            float diff = sqrt(dx * dx + dy * dy);
            if (diff > maxValue) {
                maxValue = diff;
            }
        }
        fout << "The maximum reprojection error of the " << i << " image: " << maxValue << " pixel" << std::endl;

        //找出重投影误差大于errThresh的图
        if (maxValue > errThresh) {
            outLierIndex.push_back(-1);
        } else {
            outLierIndex.push_back(0);
        }
    }
    fout.close();
    return 0;
}

CameraCalibrationStruct CCalibration::readCalibrationData(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        LOG::error("Can not read camera calibration data in " + filename + ",use default param!");
        return CameraCalibrationStruct();
    } else {
        LOG::info("Read camera calibration data success in " + filename);
        try {
            CameraCalibrationStruct calibrationStruct;
            fs["IntrinsicParameters_fx"] >> calibrationStruct.fx;
            fs["IntrinsicParameters_fy"] >> calibrationStruct.fy;
            fs["IntrinsicParameters_u0"] >> calibrationStruct.u0;
            fs["IntrinsicParameters_v0"] >> calibrationStruct.v0;
            fs["DistortionFactor_k1"] >> calibrationStruct.k_1;
            fs["DistortionFactor_k2"] >> calibrationStruct.k_2;
            fs["DistortionFactor_k3"] >> calibrationStruct.k_3;
            fs["DistortionFactor_p1"] >> calibrationStruct.p_1;
            fs["DistortionFactor_p2"] >> calibrationStruct.p_2;
            fs.release();
            return calibrationStruct;
        } catch (cv::Exception &e) {
            fs.release();
            LOG::error(e.msg.substr(0, e.msg.length() - 1));
            LOG::error("catch exception while reading calibration data! Use default param!");
            return CameraCalibrationStruct();
        }
    }
}

void CCalibration::printCalibrationData(CameraCalibrationStruct calibrationData) {
    LOG::debug("CameraCaliData.fx= " + std::to_string(calibrationData.fx));
    LOG::debug("CameraCaliData.fy= " + std::to_string(calibrationData.fy));
    LOG::debug("CameraCaliData.u0= " + std::to_string(calibrationData.u0));
    LOG::debug("CameraCaliData.v0= " + std::to_string(calibrationData.v0));
    LOG::debug("CameraCaliData.k1= " + std::to_string(calibrationData.k_1));
    LOG::debug("CameraCaliData.k2= " + std::to_string(calibrationData.k_2));
    LOG::debug("CameraCaliData.p1= " + std::to_string(calibrationData.p_1));
    LOG::debug("CameraCaliData.p2= " + std::to_string(calibrationData.p_2));
    LOG::debug("CameraCaliData.k3= " + std::to_string(calibrationData.k_3));
}

void CCalibration::run() {
    bool readSuccess = readPatternImg();
    if (!readSuccess) {
        LOG::error("Fail!  No Pattern images !");
        return;
    }
    calibProcess();
    writeParams();
}

//undistort
//校正畸变图像类
bool CUndistort::readParams() {
    CameraCalibrationStruct calibrationData;
    calibrationData = CCalibration::readCalibrationData(calibResultPath + caliCameraDataName);
    K.at<float>(0, 0) = calibrationData.fx;
    K.at<float>(1, 1) = calibrationData.fy;
    K.at<float>(0, 2) = calibrationData.u0;
    K.at<float>(1, 2) = calibrationData.v0;
    discoeff.at<float>(0, 0) = calibrationData.k_1;
    discoeff.at<float>(1, 0) = calibrationData.k_2;
    discoeff.at<float>(2, 0) = calibrationData.p_1;
    discoeff.at<float>(3, 0) = calibrationData.p_2;
    discoeff.at<float>(4, 0) = calibrationData.k_3;
    return true;
}

bool CUndistort::undistProcess() {
    //***************畸变校正****************//
    R = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
    cv::Mat mapx, mapy;
    cv::Mat srcImg = cv::imread(srcImgPath);
    cv::Mat dstImg;

    if (srcImg.empty()) {
        LOG::error("can not load repair image!");
        return false;
    }
    cv::initUndistortRectifyMap(K, discoeff, R, K, srcImg.size(), CV_32FC1, mapx, mapy);

    cv::remap(srcImg, dstImg, mapx, mapy, cv::INTER_LINEAR);
    cv::resize(dstImg, dstImg, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR);
    cv::namedWindow("Repair", 1);
    cv::imshow("Repair", dstImg);
    cv::imwrite(dstImgPath, dstImg);
    cv::waitKey(1);
    return true;
}

void CUndistort::run() {
    bool readSuccess = readParams();
    if (!readSuccess) {
        LOG::error("read Params Failed!");
    }
    undistProcess();
}

//用于摄像头标定的外部调用接口
bool CCalibration::cameraCali() {
    //标定图存储的地址
    std::string patternImgPath = PATTERN_IMG_PATH;
    FileOperation::createDirectory(patternImgPath);
    //标定相机参数文件存储的地址
    std::string calibResultPath = CALI_RESULTS_PATH;
    FileOperation::createDirectory(calibResultPath);
    //需校正图存储的地址
    std::string srcImgPath = REPAIR_IMG_PATH;
    FileOperation::createDirectory(srcImgPath);
    //校正图完成存储的地址
    std::string dstImgPath = REPAIR_IMG_PATH;
    FileOperation::createDirectory(dstImgPath);
    //标定相机参数文件名
    std::string caliCameraDataName = CALI_CAMERA_DATA_NAME;
    //标定图像素偏差参数文件名
    std::string caliBiasDataName = CALI_BIAS_DATA_NAME;
    //标定棋盘图像的行列数
    cv::Size boardSize = cv::Size(11, 8);
    //摄像头标定类初始化
    CCalibration calibration(patternImgPath, calibResultPath, caliCameraDataName, caliBiasDataName, boardSize);
    //开始摄像头标定
    calibration.run();
    //畸变图像修复类初始化
    CUndistort undistort(srcImgPath + REPAIR_IMG_NAME, dstImgPath + REPAIR_FINISH_IMG_NAME, calibResultPath, caliCameraDataName);
    //畸变图像的修复
    undistort.run();
    cv::destroyWindow("CirclePattern");
    cv::destroyWindow("Repair");
    return true;
}
