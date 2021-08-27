#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "tool/Conf.h"
#include "Infra/Thread.h"
#include "GenICam/Frame.h"
#include "serial/CRC.h"
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <cstdio>

#ifdef _LINUX
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#endif
#define RECEIVE_SIZE 18
#define SEND_SIZE 16

#define AT_PREVIOUS 0
#define AT_PRESENT  1

//32�������ݽṹ��
struct Stm32CmdStruct {
    FormatTrans32Struct pitchData;
    FormatTrans32Struct yawData;
    FormatTrans16Struct CNTR;
    uint8_t mainControlCmd;
    ShootMode shootMode;
    DistinguishMode distinguishMode;
    BulletType bulletType;
    EnemyColor enemyType;
    uint8_t shootSpeed;
    CarType carType;
    BuffBias buffBias;
};

//TX2�������ݽṹ��
struct TX2CmdStruct {
//    Point3FUnionStruct armorCoordinate;
//    Point3FUnionStruct armorCoordinateReal;
    FormatTrans32Struct pitchData;
    FormatTrans32Struct yawData;
    FormatTrans32Struct barrelToArmor;
    bool getOrderFlag = false;
    bool captureCmd = false;
    bool sameTargetFlag = false;
    FormatTrans32Struct distinguishTime;
    FormatTrans16Struct feedbackCNTR;
};

class Serial : public Dahua::Infra::CThread {
public:
    Serial();

    bool start();

    void stop();

    /**
     * TX2��������װ��
     * @param tx2Cmd �������ݽṹ��
     * @return �ɹ���־
     */
    bool sendCmd(TX2CmdStruct *tx2Cmd);

    Stm32CmdStruct getSTM32Cmd() {
        return _stm32CmdData[AT_PREVIOUS];
    }

    bool getCalReady() {
        return calReady_;
    }

    BuffBias getBuffBias() {
        return buffBias;
    }

    void resetBuffBais() {
        buffBias = NO_CHANGE;
    }

    int isLost() {
        return _isLost;
    }

    void setCalReady(bool iReady) {
        calReady_ = iReady;
    }

private:

    BuffBias buffBias;
    unsigned char _receiveData[RECEIVE_SIZE];
    unsigned char _sendData[SEND_SIZE];
    Stm32CmdStruct _stm32CmdData[2];
    std::vector<uchar> _FIFOBuffer;
    std::vector<uchar> _realData;
    bool calReady_ = false;
    bool m_isLoop;
    double calculationFps_;
    bool _isLost = false;
    int _fd2car;
    bool openSerialFlag = true;
    /**
     * ������Ϣ���Ȼ�ȡ������Ϣ
     * @param data �����յ�����Ϣ
     * @param size ��Ϣ����
     */
    void preVerifyData(const unsigned char *data, size_t size);

    /**
     * ���մ�����Ϣ
     */
    void paraReceiver();

    /**
     * ����������н��
     * @param array ������
     */
    void visionCmdDecoding(uint8_t array);

    /**
     * ��ô���������һ���֣�
     * @param array ��������
     */
    void getRealData(unsigned char *array);

    /**
     * �Ѵ������ݽ��
     * @param array ������Ϣ
     */
    void receive(unsigned char *array);

    /**
     * �����ƫ����н��
     */
    void deCodeBuffBias();

    /**
     * ���㴮���߳�ÿ�������ٶ�
     */
    void calFps();

    /**
     * �򿪴���
     * @param dev_name ��������
     * @return �ɹ���־
     */
    int openPort(const char *dev_name);

    /**
     * ���ô���
     * @param fd �������
     * @return �ɹ���־
     */
    int configurePort(int fd);

    /**
     * ���ƴ�����Ϣ
     */
    void copySTM32Cmd();

    /**
     * �߳�ִ����
     */
    void threadProc();
};

typedef Dahua::Memory::TSharedPtr<Serial> SerialPtr;
#endif
