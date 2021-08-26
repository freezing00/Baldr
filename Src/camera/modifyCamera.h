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
        OFF,        //�ر�
        ONCE,       //����
        CONTINUOUS  //�Զ�
    } CameraModeSet;

    /**
     * �����������
     */
    typedef enum {
        INIT = 0,
        DISCOVERY_CAMERA,
        START_FAIL,
        START_SUCCESS,
    }CameraInitFlow;

    /**
     * ����������ع�ģʽ
     * @param cameraSptr ����ӿ�
     * @param exposureTimeMode  ģʽ
     */
    static void modifyCameraExposureMode(Dahua::GenICam::ICameraPtr &cameraSptr, CameraModeSet exposureTimeMode);

    /**
     * ��������İ�ƽ��ģʽ
     * @param cameraSptr ����ӿ�
     * @param balanceWhiteModeSet  ģʽ
     */
    static void modifyCameraBalanceWhiteMode(Dahua::GenICam::ICameraPtr &cameraSptr, CameraModeSet balanceWhiteModeSet);

    /**
     * �����������
     * @param cameraSptr ����ӿ�
     * @param value 0-100
     */
    static void modifyCameraBrightness(Dahua::GenICam::ICameraPtr &cameraSptr, int value);

    /**
     * ��������Զ����
     * @param cameraSptr
     */
    static void modifyCameraSharpnessAuto(Dahua::GenICam::ICameraPtr &cameraSptr);

    /**
     * ��������ع�ʱ��
     * @param cameraSptr ����ӿ�
     * @param exposureTimeSet �ع�ֵ
     * @param addFlag ֱ�Ӹ�ֵ������ֵ��true->ԭֵ���ӣ�false->�趨Ϊ��ֵ��
     */
    static void modifyCameraExposureTime(Dahua::GenICam::ICameraPtr &cameraSptr, double exposureTimeSet, bool addFlag);

    /**
     * �����������
     * @param cameraSptr ����ӿ�
     * @param gainRawSet ����ֵ
     * @param addFlag ֱ�Ӹ�ֵ������ֵ��true->ԭֵ���ӣ�false->�趨Ϊ��ֵ��
     */
    static void modifyCameraGainRaw(Dahua::GenICam::ICameraPtr &cameraSptr, double gainRawSet, bool addFlag);

    /**
     * �������gamma
     * @param cameraSptr ����ӿ�
     * @param gammaSet ٤��ֵ
     * @param addFlag ֱ�Ӹ�ֵ������ֵ��true->ԭֵ���ӣ�false->�趨Ϊ��ֵ��
     */
    static void modifyCameraGamma(Dahua::GenICam::ICameraPtr &cameraSptr, double gammaSet, bool addFlag);

    //��������ֱ���
    //����ֱ������ò���Ϊ16��ֻ����1920*1280��ȥ16����������Ϊ�����Ĳ���
    static void setCameraResolution(const Dahua::GenICam::ICameraPtr &CameraSptr, int width, int height);

    //���ļ������������
    static void modifyCameraAutoSetFromFile(Dahua::GenICam::ICameraPtr &cameraSptr, DistinguishMode distinguishMode = TX2_DISTINGUISH_ARMOR);

    /**
     * �Զ������ع�ʱ��
     * @param src �����ȡ����ͼƬ
     * @param grayAverage Ŀ��ƽ���Ҷ�
     * @param targetCenter ʶ�𵽵�װ�װ����ĵ�����
     * @param targetSize ʶ�𵽵�װ�װ��С
     * @param distinguishMode ʶ��ģʽ
     * @param cameraSptr ��������ӿ�
     * @param offset ƫ����
    */
    static void autoModifyExposureTime(cv::Mat src, int grayAverage, const cv::Point2f &targetCenter, const cv::Size2f &targetSize, DistinguishMode distinguishMode, Dahua::GenICam::ICameraPtr &cameraSptr, int offset = 1);

    /**
     * ������ó�ʼ��
     * @param cameraSptr
     */
    static void modifyCameraInit(Dahua::GenICam::ICameraPtr &cameraSptr);

};

#endif
