#ifndef _MODIFY_CAMERA_H_
#define _MODIFY_CAMERA_H_

#include "tool/Conf.h"
#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/StreamSource.h"
#include <string>
#include <fstream>


#define ADD_VALUE true
#define SET_VALUE false

class ModifyCamera {
private:
    static ModifyCamera &instance() {
        static ModifyCamera modifyCamera;
        return modifyCamera;
    }

public:
    typedef enum {
        OFF,        //关闭
        ONCE,       //单次
        CONTINUOUS  //自动
    } CameraModeSet;

    /**
     * 相机连接流程
     */
    typedef enum {
        INIT = 0,
        DISCOVERY_CAMERA,
        START_FAIL,
        START_SUCCESS,
    }CameraInitFlow;

    /**
     * 设置相机的曝光模式
     * @param cameraSptr 相机接口
     * @param exposureTimeMode  模式
     */
    static void modifyCameraExposureMode(Dahua::GenICam::ICameraPtr &cameraSptr, CameraModeSet exposureTimeMode);

    /**
     * 设置相机的白平衡模式
     * @param cameraSptr 相机接口
     * @param balanceWhiteModeSet  模式
     */
    static void modifyCameraBalanceWhiteMode(Dahua::GenICam::ICameraPtr &cameraSptr, CameraModeSet balanceWhiteModeSet);

    /**
     * 设置相机亮度
     * @param cameraSptr 相机接口
     * @param value 0-100
     */
    static void modifyCameraBrightness(Dahua::GenICam::ICameraPtr &cameraSptr, int value);

    /**
     * 设置相机自动锐度
     * @param cameraSptr
     */
    static void modifyCameraSharpnessAuto(Dahua::GenICam::ICameraPtr &cameraSptr);

    /**
     * 调整相机曝光时间
     * @param cameraSptr 相机接口
     * @param exposureTimeSet 曝光值
     * @param addFlag 直接赋值或是增值（true->原值增加，false->设定为此值）
     */
    static void modifyCameraExposureTime(Dahua::GenICam::ICameraPtr &cameraSptr, double exposureTimeSet, bool addFlag);

    /**
     * 调整相机增益
     * @param cameraSptr 相机接口
     * @param gainRawSet 增益值
     * @param addFlag 直接赋值或是增值（true->原值增加，false->设定为此值）
     */
    static void modifyCameraGainRaw(Dahua::GenICam::ICameraPtr &cameraSptr, double gainRawSet, bool addFlag);

    /**
     * 调整相机gamma
     * @param cameraSptr 相机接口
     * @param gammaSet 伽马值
     * @param addFlag 直接赋值或是增值（true->原值增加，false->设定为此值）
     */
    static void modifyCameraGamma(Dahua::GenICam::ICameraPtr &cameraSptr, double gammaSet, bool addFlag);

    //设置相机分辨率
    //相机分斌率设置步长为16，只能用1920*1280减去16的整数倍作为函数的参数
    static void setCameraResolution(const Dahua::GenICam::ICameraPtr &CameraSptr, int width, int height);

    //从文件设置相机参数
    static void modifyCameraAutoSetFromFile(Dahua::GenICam::ICameraPtr &cameraSptr, DistinguishMode distinguishMode = TX2_DISTINGUISH_ARMOR);

    /**
     * 自动调整曝光时间
     * @param src 相机获取到的图片
     * @param grayAverage 目标平均灰度
     * @param targetCenter 识别到的装甲板中心点坐标
     * @param targetSize 识别到的装甲板大小
     * @param distinguishMode 识别模式
     * @param cameraSptr 相机操作接口
     * @param offset 偏移量
    */
    static void autoModifyExposureTime(cv::Mat src, int grayAverage, const cv::Point2f &targetCenter, const cv::Size2f &targetSize, DistinguishMode distinguishMode, Dahua::GenICam::ICameraPtr &cameraSptr, int offset = 1);

    /**
     * 相机设置初始化
     * @param cameraSptr
     */
    static void modifyCameraInit(Dahua::GenICam::ICameraPtr &cameraSptr);

};

#endif
