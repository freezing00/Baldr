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

//32控制数据结构体
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

//TX2控制数据结构体
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
     * TX2发送数据装填
     * @param tx2Cmd 发送数据结构体
     * @return 成功标志
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
     * 根据信息长度获取串口信息
     * @param data 串口收到的信息
     * @param size 信息长度
     */
    void preVerifyData(const unsigned char *data, size_t size);

    /**
     * 接收串口信息
     */
    void paraReceiver();

    /**
     * 对主命令进行解包
     * @param array 主命令
     */
    void visionCmdDecoding(uint8_t array);

    /**
     * 获得串口命令（解包一部分）
     * @param array 串口命令
     */
    void getRealData(unsigned char *array);

    /**
     * 把串口数据解包
     * @param array 串口信息
     */
    void receive(unsigned char *array);

    /**
     * 对神符偏差进行解包
     */
    void deCodeBuffBias();

    /**
     * 计算串口线程每秒运行速度
     */
    void calFps();

    /**
     * 打开串口
     * @param dev_name 串口名称
     * @return 成功标志
     */
    int openPort(const char *dev_name);

    /**
     * 设置串口
     * @param fd 串口序号
     * @return 成功标志
     */
    int configurePort(int fd);

    /**
     * 复制串口信息
     */
    void copySTM32Cmd();

    /**
     * 线程执行体
     */
    void threadProc();
};

typedef Dahua::Memory::TSharedPtr<Serial> SerialPtr;
#endif
