#include "serial.h"

#define DEBUG_SERIAL_CHANGE 0
#define DEBUG_SERIAL_NO_CHANGE 0
#define SHOW_SERIAL_TIME 0
#define DEBUG_TX2_SEND 0
#define DEBUG_TX2_SEND_NO_CHANGE 0

Serial::Serial()
        : Dahua::Infra::CThread("serial") {
    setCalReady(false);
    _FIFOBuffer.clear();
    _realData.clear();
    _realData.clear();
    for (int i = 0; i < RECEIVE_SIZE; i++)
        _realData.push_back(0);
}

bool Serial::start() {
    m_isLoop = true;
    return Dahua::Infra::CThread::createThread();
}

void Serial::stop() {
    m_isLoop = false;
#ifdef _LINUX
    close(_fd2car);
#endif
}

void Serial::calFps() {
    static int count = 0;
    static double lastTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000; // ms
    ++count;
    // 取固定帧数为100帧计算一次
    if (count >= 100) {
        double curTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000;
        calculationFps_ = count / (curTime - lastTime) * 1000;
        lastTime = curTime;
        count = 0;
    }
}

int Serial::openPort(const char *dev_name) {
    int fd = -1; // file description for the serial port
#ifdef _LINUX
    fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        // if open is unsucessful
        LOG::info("open_port: Unable to open " + LOG::tostring(dev_name));
    } else {
        fcntl(fd, F_SETFL, FNDELAY);
        LOG::info("port is open.");
    }
#endif
    return fd;
}

int Serial::configurePort(int fd) {
#ifdef _LINUX
    struct termios port_settings;               // structure to store the port settings in
    cfsetispeed(&port_settings, B460800);       // set baud rates
    cfsetospeed(&port_settings, B460800);
    /* Enable the receiver and set local mode...*/
    port_settings.c_cflag |= (CLOCAL | CREAD);
    /* Set c_cflag options.*/
    port_settings.c_cflag &= ~PARENB;           // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~PARODD;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;
    //port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    port_settings.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    port_settings.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP);
    /* Set c_oflag output options */
    port_settings.c_oflag &= ~OPOST;
    /* Set the timeout options */
    port_settings.c_cc[VTIME] = 0;
    port_settings.c_cc[VMIN] = 0;
    tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port
#endif
    return fd;
}

void Serial::preVerifyData(const unsigned char *data, size_t size) {
    static uint8_t step = 0;
    if (size > 0) {
        for (int i = 0; i < size; i++) {
            _FIFOBuffer.push_back(data[i]);
        }
    }
    if (_FIFOBuffer.size() > 0) {
        switch (step) {
            case 0:
                while (!(_realData[0] == 0xD4 && _realData[1] == RECEIVE_SIZE)) {
                    if (_FIFOBuffer.size() > 0) {
                        _realData.push_back(_FIFOBuffer[0]);
                        _FIFOBuffer.erase(std::begin(_FIFOBuffer));
                        _realData.erase(std::begin(_realData));

                    } else {
                        break;
                    }
                }
                step = 1;
                break;
            case 1:
                if ((_realData[0] == 0xD4) && _realData[1] == RECEIVE_SIZE) {
                    for (int i = 0; i < RECEIVE_SIZE; i++) {
                        _receiveData[i] = _realData[i];
#if DEBUG_SERIAL_NO_CHANGE
                        printf("%x ", _realData[i]);
#endif
                    }
#if DEBUG_SERIAL_NO_CHANGE
                    cout << endl;
#endif
                    if (_FIFOBuffer.size()) {
                        _realData.push_back(_FIFOBuffer[0]);
                        _FIFOBuffer.erase(std::begin(_FIFOBuffer));
                        _realData.erase(std::begin(_realData));
                    }
                    step = 0;
                } else {
                    if (_FIFOBuffer.size()) {
                        _realData.push_back(_FIFOBuffer[0]);
                        _FIFOBuffer.erase(std::begin(_FIFOBuffer));
                        _realData.erase(std::begin(_realData));
                    }
                    step = 0;
                }
                break;
        }
    } else {
        return;
    }
}

void Serial::receive(unsigned char *array) {
    static int seq = 0;
    static uint8_t successCNT = 0;
#if SHOW_SERIAL_TIME
    static double t = 0.0;
#endif
    if (!CRC::verifyCRC8CheckSum(array, 3) && !CRC::verifyCRC16CheckSum(array, array[1])) {
        setCalReady(false);
    } else {
        //必须保证在通过CRC校验的情况下，包序号和上一时间的不一致才对结构体进行更新
        if (seq != ((int) (array[15] << 8) | array[14])) {
#if SHOW_SERIAL_TIME
            double time = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            t = (double)cv::getTickCount();
            cout << "time:" << time << endl;
#endif
            //复制串口信息
            copySTM32Cmd();
            //获取串口真正指令
            getRealData(array);
            //在接收到5次信息之后才会设置串口准备完成
            if (successCNT < 5) {
                successCNT++;
            } else {
                setCalReady(true);
            }
        }
        seq = (int) (array[15] << 8) | array[14];
    }
}

void Serial::paraReceiver() {
    unsigned char buf[255] = {0};
    size_t bytes = 0;
#ifdef _LINUX
    ioctl(_fd2car, FIONREAD, &bytes);

    static int noDataNum = 0;
    if(bytes == 0){
        noDataNum++;
    }
    if(bytes > 0)
        noDataNum = 0;
    else if(noDataNum > 1000){
        _isLost = 1;
    }

    if (bytes > 32) {
        bytes = read(_fd2car, (unsigned char *) buf, 32);
    } else if (bytes <= 32) {
        bytes = read(_fd2car, (unsigned char *) buf, bytes);
    }
#endif
    //初步校验串口信息
    preVerifyData(buf, bytes);
}

bool Serial::sendCmd(TX2CmdStruct *tx2Cmd) {
#ifdef _LINUX
    if(openSerialFlag){
        uchar index = 0;
        uchar ptr_index = 3;
        _sendData[0] = 0xD4;
        //命令帧
        _sendData[ptr_index++] = (tx2Cmd->getOrderFlag << 2) | (tx2Cmd->sameTargetFlag << 1) | tx2Cmd->captureCmd;

        //坐标帧
        //for (index = 0; index < 2; index++)
        //    _sendData[ptr_index++] = tx2Cmd->armorCoordinate.x.u8_temp[index];
        //for (index = 0; index < 2; index++)
        //    _sendData[ptr_index++] = tx2Cmd->armorCoordinate.y.u8_temp[index];
        //for (index = 0; index < 2; index++)
        //    _sendData[ptr_index++] = tx2Cmd->armorCoordinate.z.u8_temp[index];
        //绝对坐标帧
        //for (index = 0; index < 2; index++)
        //    _sendData[ptr_index++] = tx2Cmd->armorCoordinateReal.x.u8_temp[index];
        //for (index = 0; index < 2; index++)
        //    _sendData[ptr_index++] = tx2Cmd->armorCoordinateReal.y.u8_temp[index];
        //for (index = 0; index < 2; index++)
        //    _sendData[ptr_index++] = tx2Cmd->armorCoordinateReal.z.u8_temp[index];

        //牛顿迭代法算出的pitch和yaw角度
        for (index = 0; index < 4; index++)
            _sendData[ptr_index++] = tx2Cmd->yawData.u8_temp[index];
        for (index = 0; index < 4; index++)
            _sendData[ptr_index++] = tx2Cmd->pitchData.u8_temp[index];

        //回返包序号
        for (index = 0; index < 2; index++)
            _sendData[ptr_index++] = tx2Cmd->feedbackCNTR.u8_temp[index];
        _sendData[1] = ptr_index + 2;
        CRC::appendCRC8CheckSum(_sendData, 3);
        CRC::appendCRC16CheckSum(_sendData, _sendData[1]);
    #if DEBUG_TX2_SEND
        //cout<<"getOrderFlag:"<<tx2Cmd->getOrderFlag<<endl;
        //cout<<"sameTargetFlag:"<<tx2Cmd->sameTargetFlag<<endl;
        cout<<"captureCmd:"<<tx2Cmd->captureCmd<<endl;
        //cout<<"armorCoordinate.x:"<<tx2Cmd->armorCoordinate.x.s16_temp<<endl;
        //cout<<"armorCoordinate.y:"<<tx2Cmd->armorCoordinate.y.s16_temp<<endl;
        //cout<<"armorCoordinate.z:"<<tx2Cmd->armorCoordinate.z.s16_temp<<endl;
        //cout<<"armorCoordinateReal.x:"<<tx2Cmd->armorCoordinateReal.x.s16_temp<<endl;
        //cout<<"armorCoordinateReal.y:"<<tx2Cmd->armorCoordinateReal.y.s16_temp<<endl;
        //cout<<"armorCoordinateReal.z:"<<tx2Cmd->armorCoordinateReal.z.s16_temp<<endl;
        //cout<< "yawData:" << tx2Cmd->yawData.float_temp << endl;
        //cout<<"pitchData:"<<tx2Cmd->pitchData.float_temp<<endl;
        //cout<<"CNTR:"<<tx2Cmd->feedbackCNTR.u16_temp<<endl<<endl;
    #endif
    #if DEBUG_TX2_SEND_NO_CHANGE
            for(int i=0;i<_sendData[1];i++){
                printf("%d ",_sendData[i]);
            }
            cout<<endl;
    #endif
        //发送数据
        if (_sendData[1] == write(_fd2car, _sendData, _sendData[1]))
            return true;
        else
            return false;
    }
    else
        return false;
#endif
}

void Serial::visionCmdDecoding(uint8_t array) {
    _stm32CmdData[AT_PRESENT].distinguishMode = (DistinguishMode)(array & 0x07);
    _stm32CmdData[AT_PRESENT].enemyType = (EnemyColor) ((array & 0x08) >> 3);
    _stm32CmdData[AT_PRESENT].buffBias = (BuffBias) (array >> 4);
    //对神符偏差进行解包
    deCodeBuffBias();
}

void Serial::deCodeBuffBias() {
    static int count = 0;
    if (_stm32CmdData[AT_PRESENT].buffBias != NO_CHANGE) {
        count++;
    } else {
        count = 0;
    }
    if (count >= 2) {
        _stm32CmdData[AT_PRESENT].buffBias = NO_CHANGE;
    }
    if (_stm32CmdData[AT_PRESENT].buffBias != NO_CHANGE) {
        buffBias = _stm32CmdData[AT_PRESENT].buffBias;
    }
}

void Serial::getRealData(unsigned char *array) {
    unsigned char index = 0;
    uint8_t cnt = 3;
    _stm32CmdData[AT_PRESENT].mainControlCmd = array[cnt++];
    _stm32CmdData[AT_PRESENT].carType = (CarType) (array[cnt++]); //车辆编号
    _stm32CmdData[AT_PRESENT].shootSpeed = (array[cnt++]);       //射速

    for (index = 0; index < 4; index++)
        _stm32CmdData[AT_PRESENT].pitchData.u8_temp[index] = array[cnt++];
    for (index = 0; index < 4; index++)
        _stm32CmdData[AT_PRESENT].yawData.u8_temp[index] = array[cnt++];
    for (index = 0; index < 2; index++)
        _stm32CmdData[AT_PRESENT].CNTR.u8_temp[index] = array[cnt++];

    //对主命令进行解包
    visionCmdDecoding(_stm32CmdData[AT_PRESENT].mainControlCmd);
}

void Serial::copySTM32Cmd() {
    _stm32CmdData[AT_PREVIOUS].pitchData.float_temp = _stm32CmdData[AT_PRESENT].pitchData.float_temp;
    _stm32CmdData[AT_PREVIOUS].yawData.float_temp = _stm32CmdData[AT_PRESENT].yawData.float_temp;
    _stm32CmdData[AT_PREVIOUS].mainControlCmd = _stm32CmdData[AT_PRESENT].mainControlCmd;
    _stm32CmdData[AT_PREVIOUS].CNTR.u16_temp = _stm32CmdData[AT_PRESENT].CNTR.u16_temp;
    _stm32CmdData[AT_PREVIOUS].distinguishMode = _stm32CmdData[AT_PRESENT].distinguishMode;
    _stm32CmdData[AT_PREVIOUS].enemyType = _stm32CmdData[AT_PRESENT].enemyType;
    _stm32CmdData[AT_PREVIOUS].shootSpeed = _stm32CmdData[AT_PRESENT].shootSpeed;
    _stm32CmdData[AT_PREVIOUS].buffBias = _stm32CmdData[AT_PRESENT].buffBias;
    _stm32CmdData[AT_PREVIOUS].carType = _stm32CmdData[AT_PRESENT].carType;
    //_stm32CmdData[AT_PREVIOUS].bulletType = _stm32CmdData[AT_PRESENT].bulletType;
    //_stm32CmdData[AT_PREVIOUS].shootMode = _stm32CmdData[AT_PRESENT].shootMode;
#if DEBUG_SERIAL_CHANGE
    cout << "serialSize: " << _FIFOBuffer.size() << endl;
    cout << "pitchAngle:" << _stm32CmdData[AT_PREVIOUS].pitchData.float_temp << endl;
    cout << "yawAngle:" << _stm32CmdData[AT_PREVIOUS].yawData.float_temp << endl;
    cout << "CNTR:" << _stm32CmdData[AT_PREVIOUS].CNTR.u16_temp << endl;
    cout << "distinguishMode:" << (int)_stm32CmdData[AT_PREVIOUS].distinguishMode << endl;
    cout << "enemyType::" << _stm32CmdData[AT_PREVIOUS].enemyType << endl;
    cout << "shootSpeed:" << (int)_stm32CmdData[AT_PREVIOUS].shootSpeed << endl;
    cout << "buffBias:" << _stm32CmdData[AT_PREVIOUS].buffBias << endl << endl;
    cout << "carType:" << (int)_stm32CmdData[AT_PREVIOUS].carType <<endl;
    //cout << "bulletType:" << _stm32CmdData[AT_PREVIOUS].bulletType << endl;
    //cout << "shootMode:" << _stm32CmdData[AT_PREVIOUS].shootMode << endl;
#endif
}

void Serial::threadProc() {
#ifdef _LINUX

    if (!access("/dev/ttyUSB0", F_OK)) {
        _fd2car = openPort("/dev/ttyUSB0");
        LOG::info("ttyUSB0");
    } else if (!access("/dev/ttyUSB1", F_OK)) {
        _fd2car = openPort("/dev/ttyUSB1");
        LOG::info("ttyUSB1");
    } else

    {
        openSerialFlag=false;
        LOG::info("No serial");
        _isLost = true;
    }
    //_fd2car = openPort("/dev/ttyTHS2");
#endif
    if (openSerialFlag) {
        configurePort(_fd2car);
        while (m_isLoop) {
            //获取串口的数据
            paraReceiver();
            //把串口数据解包
            receive(_receiveData);
            //calFps();                                                               //计算FPS
            
#ifdef _LINUX
            usleep(10);                           //挂起100us
#endif
        }
    }
}
