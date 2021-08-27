# Baldr

## 目录

* [功能介绍](#功能介绍)

* [安装说明](#安装说明)

* [详细算法](#详细算法)

* [通讯协议](#通讯协议)

* [参与贡献](#参与贡献)

---

## 功能介绍

此代码为桂林电子科技大学Evolution战队2021年的视觉识别代码，主要用于识别装甲板以及能量机关。在WIN64上测试，Ubuntu20.04上运行。
本代码统一使用的大华工业相机分辨率1920*1200，帧率150，焦距8mm和12mm镜头

---

## 安装说明

  1. 在WIN64上运行需要Visual Studio 2019，在Ubuntu上运行需要gcc。
  2. 需要安装OpenCV4.5.0
  3. 需要安装大华相机驱动（MV viewer 2.2.5） [下载地址](http://download.huaraytech.com/pub/sdk/Ver2.2.5/)
  4. 解决方案配置使用Release
---

## 详细算法

### 文件结构

```
├─include									//包含路径
├─build										//生成路径
├─log										//日志文件
├─Src
│  │  main.cpp								//主函数（拉流，创建线程）
│  │      
│  ├─angle
│  │  │  angleFactory.cpp					//角度解算服务
│  │  │  angleFactory.h
│  │  │  
│  │  ├─CoordinateSolver
│  │  │      ArmorCoordinateSolver.cpp		//装甲板解算
│  │  │      ArmorCoordinateSolver.h
│  │  │      BuffCoordinateSolver.cpp		//神符解算
│  │  │      BuffCoordinateSolver.h
│  │  │      
│  │  ├─KalmanPredict
│  │  │	KalmanPredict.cpp
│  │  │	KalmanPredict.h
│  │  │ 
│  │  └─PNP
│  │          PNPSolver.cpp					//PNP解算
│  │          PNPSolver.h
│  ├─armor
│  │  │  armorDistinguish.cpp				//装甲板识别
│  │  │  armorDistinguish.h
│  │  │  
│  │  └─bpPredict
│  │       net.cpp
│  │       net.h
│  │      
│  ├─buff
│  │      buffDistinguish.cpp				//能量机关识别
│  │      buffDistinguish.h
│  │      buffTest.cpp
│  │      buffTest.h
│  │      
│  ├─camera
│  │  │  modifyCamera.cpp					//相机设置工具类
│  │  │  modifyCamera.h
│  │  │  streamRetrieve.cpp					//相机拉流
│  │  │  streamRetrieve.h
│  │  │  
│  │  └─calibration
│  │          cameraCalibration.cpp
│  │          cameraCalibration.h
│  │      
│  ├─decisionLevel
│  │      decisionLevel.cpp					//决策层
│  │      decisionLevel.h
│  │      
│  ├─serial
│  │      CRC.h								//CRC工具
│  │      serial.cpp						//串口
│  │      serial.h
│  │      
│  └─tool
│          autoSaveSample.cpp				//自动保存机器学习样本
│          autoSaveSample.h
│          Conf.h							//配置文件
│          fileOperation.cpp				//文件操作辅助类
│          fileOperation.h
│          PointUtil.h						//常用操作工具类
│          RMDefine.h						//常用定义
│          RMLOG.h							//日志服务
│          systemChoose.h
│          
├─Tools                                 //工具文件夹
│  │  autoPull.sh                       //自动代码更新脚本
│  │  dog.sh                            //进程守护脚本
│  │  Baldr.sh                         //强制退出代码脚本
│  └─SVM                                //SVM训练代码
├─trainData                             //机器学习训练结果
│  │  
│  ├─armor                              //装甲板样本
│  ├─buff                               //能量机关样本
│  └─circle                             //能量机关圆心样本
├─visionData
│  │  armorData.yml                     //装甲板尺寸数据
│  │  armorPara.yml                     //装甲板识别参数
│  │  buffPara.yml                      //能量机关识别参数
│  │  cameraConfigurationData.yml       //不同识别模式下的相机参数
│  │  codeSet.yml                       //代码设置
│  │  installData.yml                   //相机安装参数
│  │  
│  ├─AVISave                            //相机录制的视频
│  ├─cameraCaliData
│  │  │  serialNumber.txt
│  │  │  标定棋盘图.jpg
│  │  │  
│  │  └─caliResults                     //相机标定结果
│  └─imgSave                            //相机拍的照片
```
### 识别流程

系统整体架构图如下，每个线程都有自己的分工。由主线程创建所有子线程并维护。
![整体流程](http://img.peterli.club/img/整体流程.png)


### 能量机关识别
#### 识别方案1

![神符识别流程图](http://img.peterli.club/img/image-20200730212019478.png)

1. 对输入的图片先进行ROI处理。根据上一帧预置的一个矩形范围进行合理的缩放来对图片进行ROI的截取。之后进行二值化处理。根据不同的颜色进行不同的阈值处理，并且进行适当的腐蚀操作完成对图像的二值化处理。
2. 对初步处理的图片进行轮廓的提取，对所有的轮廓迭代进行初步的筛选，筛掉长宽比小的矩形，并且让这些矩形进入疑似圆心矩形动态数组，之后筛掉面积过小的矩形，去掉二值化区域中的小噪点。
3. 对上一步得到的疑似圆心矩形迭代，根据面积、长宽比和SVM进行筛选，但根据实际效果来说，SVM效果并不是非常理想，因此在根据SVM筛选之前再让所有满足形态学的矩形进入动态数组，在一轮筛选过后，如果没找到圆心，再对上述矩形进行筛选，选择和上帧圆心矩形形态学符合筛选条件的矩形为圆心矩形。
4. 对2步骤中筛出来的矩形集再次进行迭代，此时我们根据嵌套和各种矩形的形态学特征对这些矩形进行分组操作，每一个组是一个结构体，实际上为已打击和待打击的悬臂的集合，每个结构体包括装甲板矩形、悬臂矩形和内嵌数目三个成员，这样的分组操作便于后续处理和维护。分完组后，我们根据每个结构体中的内嵌数量区分待打击和已打击的悬臂，待打击悬臂内嵌数明显小于已打击的，因此此时我们就得出了待打击结构体，其中要打击的装甲板即为里面的成员。
5. 选出识别到的最终矩形后，我们根据识别到的圆心缩放得到下一帧的有效矩形，并且对最终矩形和圆心坐标进行ROI还原。如果当前帧没有识别到圆心，我们则不断扩大ROI面积进行搜索，直到扩大为整张图片。
6. 在解算过程中，我们先根据目标矩形中心和圆心连线的角度变化来判断旋转方向，得知旋转方向后我们求解需要预置的角度。由于小符为匀速，我们直接预置合适的角度并加入操作手接口以便让操作手根据实际情况进行调节。对于大符来说，我们首先获取实时角速度，并且对刚开始40帧进行正弦函数拟合，再根据延迟时间（子弹飞行、云台延时等）求解积分作为预置角度。得出预置角度后我们对目标矩形的四个点相应的平移并放入PNP解算中进行解算，最终得出数据发送给下位机。
![神符二值化图片](http://img.peterli.club/img/神符二值化图片.jpg)

#### 识别方案2

##### 题目总览  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;无论解决能量机关也好其他也罢，我们要做的就是求一类问题的通解
##### 赛场建议
* 能量机关激活点观察能量机关,视觉干扰较小
* 采用传统视觉方案、图像阈值非常重要
* 能量机关识别各种限制条件尽可能放宽松些
#####  题目分析
* 目标  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;能量机关的识别目标非常明确，将能量机关的中心点找到，将需要击打的装甲板找到。
*  解决方案  
![在这里插入图片描述](https://img-blog.csdnimg.cn/1bb52b2d56434375bf26eaee45d7ac10.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQ1ODEyOTQx,size_16,color_FFFFFF,t_70#pic_center)  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;如上为能量机关识别算法方案二，在视觉项目中以BuffTest命名。
##### 关于能量机关中心
* 初始方案  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;判断大小->RotatedRect是否类似于正方形->SVM[支持向量机二分类]->得出检测结果。 
* 遇见问题  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;在寻找某个目标时故要迭代，因为SVM时二分类，如果在没有跌倒到中心R时，SVM已给出结果是R则，我们要不要继续迭代呢?迭代了也意义不大，因为有多个时，无法判断哪一个是正确目标。
* 版本迭代过程  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;在此处我们也用到了许多的各种图像识别方案。但是由于我们的理论知识有限、有些问题也不能研究透彻。 SVM->人工神经网络->相似度对比
* 最终使用方案  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;在方案一上去掉了SVM。
* 赛场效果  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;比赛中、在能量机关激活点相机视野没有什么视觉干扰，对于中心R只使用大小限制方案是能够解决的。遇见的问题:不知道中心R在相机中有多大，需要临场调试。

##### 能量机关装甲板
* 方案  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;根据R的大小为相对单位,来作为寻找装甲板轮廓以及RotatedRect大小的条件。  
* 限制条件[赛场一定要大胆放松条件]    
`1、RotatedRect的长与宽的大小以及二者的比例`  
`2、RotatedRect中心与R中心连线与RotatedRect的长边夹角、理论为90，要合理限制不要限制过死`  
`3、RotatedRect中心与R中心距离，要合理限制不要限制过死`   
* 赛场    
`1、RotatedRect的长与宽的大小问题较大，周围干扰较小`  
`2、RotatedRect中心与R中心连线与RotatedRect的长边夹角、理论为90，要合理限制不要限制过死`  

##### 怎样找出待打击
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;可见待打击的悬臂是三根灯条、以激活的悬臂为三根灯条，此方案围绕三根灯条展开。对每个悬臂上截取感性区域，再做如图处理。根据二值图的白色像素点多少即可找出待打击。
##### 关于整体图像的感性区域
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;请参考方案一



---
### 装甲板识别
####1.程序流程图
![装甲板识别流程图](http://img.peterli.club/img/装甲板.png)

#### 2.追踪器和分类器

##### 1.追踪器

由于opencv现有的八类追踪器中，KCF Tracker不论是从速度上还是准确率上都是比较优异的。使用追踪器的目的是在云台跟随一个目标时，不会随意的切换到其他目标上。使程序具备追踪的效果。

KCF原理:使用循环矩阵对样本进行采集，使用快速傅里叶变换对算法进行加速计算。

opencv3.1.0版本后的均支持KCF Tracker。
```cpp
cv::Ptr<cv::TrackerKCF> _tracker = cv::TrackerKCF::create(); //创建追踪器
void ArmorDistinguish::armorTrackerInit(const cv::Mat& src, uchar enemyColor)  //追踪器初始化
_tracker->update(_src, _trackerRect)             //更新追踪区域
```

##### 2.分类器

分类器识别数字由原先的SVM变成bp模型。SVM在训练的时候样本的训练集数量以及正负样本的类别选择比较苛刻，并且svm实现多分类的方法需要构建并训练多个分类器，所以这里采用bp神经网络模型。

bp原理:通过自身的训练，学习某种规则，在给定输入值时得到最接近期望输出值的结果。bp网络是一种按误差方向传播训练的多层前馈网络，基本思想是梯度下降，最终让网络的实际输出值和期望输出值的误差均方差最小。训练的核心算法为前向传播和反向传播。

为了保证程序的运算速度，这里bp模型的隐藏层只有一层，数字的分辨率是20*20，隐藏层的结点为64.输出层为七类，分别是数字1(英雄)，2(工程)，3(步兵)，4(步兵)，5(前哨战)， 6(基地)，7(哨兵)。整个过程包括了，样本采集，数据预处理，网络训练，训练过程把权重和偏置保存成XML的形式。

使用时，这里构造了net类使用bp模型，对传入的数据直接进行分类核心算法是前向传播。

```cpp
namespace bp {
    class Net {
    public:
        Net() {};
        ~Net() {};
        int predict_one(cv::Mat &input);
        void load(const std::string &filename);  //导入训练的xml文件
        static cv::Mat bpImgPreProcess(cv::Mat input);//对图片进行预处理
        std::vector<int> layerNeuronNum;		//网络的结构
        std::string activeFunction = "tanh";	//选择tanh激活函数
        float learningRate;						//学习率(分类时不用使用，只在训练的时候使用)
    private:
        int _resultNumber;						//分类的结果
        void forward();							//前向传播
        void initNet(std::vector<int> layer_neuron_num_); //通过XML文件初始化bp模型
    protected:
        std::vector<cv::Mat> layer;
        std::vector<cv::Mat> weights;
        std::vector<cv::Mat> bias;
        cv::Mat activationFunction(cv::Mat &x, const std::string &func_type);
        static cv::Mat sigmoid(cv::Mat &x);		//sigmoid激活函数
        static cv::Mat tanh(cv::Mat &x);		//tanh激活函数
    };
}
```
这里实现的是多分类，因此tanh激活函数会比sigmoid激活函数准确率好很多。稳定的车俩ID识别是基础，这样就可以做优先级打击序列，以及针对特定目标的打击。

![测试图1](http://img.peterli.club/img/H%7BJUCE%7DW2OHKDXUP%7D5RKY%252.png)

![测试图2](http://img.peterli.club/img/%7BL2DO%7B4LD%7BH%25SADO9%40OX%24%40A.png)



### 装甲板预判打击

#### 1、陀螺仪坐标系
将预判打击视为一个运动学问题，那么我们需要一个参考系。但相机坐标系随着相机运动而运动，在跟随目标时坐标系也发生了变化，在相机坐标系下预判难度较大。因此我们引入陀螺仪坐标系的概念。
陀螺仪是个姿态传感器，它一般固定在云台上某个远离振动源的位置。陀螺仪的作用就是反馈当前云台的角度数据（Yaw轴、Pitch轴、Roll轴角度）（当然也可以反馈角速度和加速度数据，只不过我们暂时用不到）。
以Yaw轴为例，Yaw轴角度有一个零点，在忽略陀螺仪零漂的情况下，这个零点是相对固定不会改变的。这个零点可以类比为指南针的南极，无论你如何移动它始终指向一个方向（实际上陀螺仪的Yaw轴数据一般
就是通过磁力计获得的）。如此一来我们就有了个参考——无论云台处于何种姿态，我们只需要将相机坐标系依照Yaw轴与Pitch轴的角度数据进行反向旋转，都可以得到同一个坐标系，这个坐标系就是陀螺仪坐标系。
既然我们可以进行坐标系的变换，坐标变换就易如反掌，可以通过目标在相机坐标系下的坐标获得目标在陀螺仪坐标系下的坐标。陀螺仪坐标系的原点跟随自身机器人运动，此时视原点静止，将自身机器人的运动与
目标的运动叠加在一起，即可构建物理模型进行预判打击。

#### 2、预测弹道落点
经过前面的铺垫，我们已经将预判打击转换为物理问题，只要通过简单的物理定律就可以解决。通过对坐标进行逐差法计算，可以获得目标当前的运动状态，如速度和加速度。获得了目标的运动状态后，还需要关键
的一点——预判时间。究竟这个时间该取多少才合适？如果我们的机器人射出的是激光，那么不需要预判甚至不需要计算枪口的Pitch轴角度。但我们发射的是弹丸，弹丸飞行需要时间，在弹丸飞行时目标也在移动。
想要知道弹丸飞行时间需要知道弹道落点，但现在需要知道飞行时间才能求出弹道落点，这个问题似乎陷入了死循环。
这里我们采用迭代法来求得近似的弹丸飞行时间。将弹丸飞行至当前时刻目标位置所用的时间设为迭代初值并开始迭代。计算出经过时间后目标所处的位置，求得弹丸飞行至所用时间，并用于作差得到时间差△t并
将△t作为本次迭代的步长。用加上步长得到，并将作为新的迭代初值进行下一轮迭代。当△t小于一定值或迭代次数到达上限即停止迭代，可获得最终的弹丸飞行时间。可以给△t乘上一个系数α，通过调节α的值来缩短
迭代时间，但要注意α的取值以防发散。最后用求得最终的预判坐标。

#### 3、数据处理
在预判打击的过程中肯定要进行数据处理来对抗数据误差。我们在获得陀螺仪坐标系下的坐标后，先进行一波简单的数据剔除与插值。即剔除明显错误的数据并用上一次正确的数据做插值。在完成初步处理后，对数
据进行卡尔曼滤波，保证数据平滑正确。然后再进行目标运动状态的计算，这样可以保证云台预判的流畅性。同时限于云台响应的延迟（在跟随运动目标时存在100ms左右的延时），预判时也给予了延时补偿，这个
延时补偿会造成目标在突然变向时云台反应不及（也包含云台自身惯性的原因），这个问题解决的根源还是在于云台响应。

### 能量机关(以下称为神符)预判

预判部分主要是利用识别部分的目标矩形和R矩形，在连续多帧获取数据后，判断旋转方向以及计算相应的参数，模拟出神符旋转的轨迹，计算预判出对应的提前量，将目标矩形预置到打击位置。

#### 1.初始化处理

首先以圆心为基点向X轴正方向做射线，将此射线作为基准线。然后将获取的目标矩形中心点与R矩形中心点连接成线，计算此线段与基准线的夹角。因为神符不仅自身旋转，其叶片每隔2.5S便会进行切换，所以需要对是否切换叶片进行判断。我们只需要存储神符同一片叶片的旋转数据，只要切换就将容器清空。存储五个角度数据后先逐差法求均值，因为帧率较高所以帧间角度变化不大。依据之前设定的基准线，只需要判断该平均值的正负便可判断出旋转方向。每帧利用静态变量存储上次的角度以及目标矩形中心点。

#### 2.预判算法

根据旋转方向得出帧间转角的正负。然后计算下一帧该叶片的二维旋转角度。首先计算实时的旋转角速度。定义好变量储存上一帧数据，对于角度、时间、帧数都需要储存和更新，每0.1s更新一次数据。计算角速度的同时储存最大角速度和最小角速度，当经过一定帧数后就将该最小角速度和最大角速度视为波峰和波谷速度。最后需要判断出角速度的加速和减速状态，将其存储到类成员中。其次根据角速度的加速或者减速状态定位出本帧处于轨迹函数的具体时刻。依靠按键区分大小神符，小符预置固定的提前角度；大符利用积分计算出预判角度。得出角度预置量后，计算出R矩形中心到目标矩形中心的距离作为构造的直角三角形的斜边。利用三角函数和角度预置量计算出最终的目标矩形中心点。

#### 3.数据处理

预判后目标矩形的位置得到了更新，取目标矩形的四个角点进行位姿解算测距，得到真实的世界坐标。然后通过卡尔曼滤波进行数据处理，根据是否切换叶片进行对卡尔曼滤波对象的重置。最后回到角度工厂计算出二轴角度，传输给电控。

### 反小陀螺策略

#### 1、自动识别敌方状态

基于自瞄状态下，通过历史数据判断当前跟踪的装甲是否是同一块装甲板。如果装甲板出现切换，则判断切换前后固有角度的变化，通过两次的角度变化判断旋转方向。当同一旋转方向的次数达到一定次数进入反陀螺模式。反陀螺模式下，识别同一块装甲板的次数或者识别丢失会取消反陀螺模式，重新进入判别是否进入反陀螺模式。

#### 2、反小陀螺模式

因为装甲板切换是在陀螺旋转时出现，所以记录下切换前的固有坐标和切换后的固有坐标。对记录的前后坐标进行多次储存，最后对储存的坐标取平均值获得的固有坐标基本上会识别为敌方陀螺的中心位置。只有固有坐标超过最小和最大的固有坐标才会使识别的位置发生变化。




## 通讯协议
### 1.通信方式

使用USB串口与32进行通信，并经过CRC校验

子弹弹道以及云台角度的解算在32端完成。

### 2.通信接收结构体

```cpp
typedef struct{
    FormatTrans32Struct pitchData;		//云台pitch轴角度
    FormatTrans32Struct yawData;		//云台yaw轴角度
    FormatTrans16Struct CNTR;			//CRC校验码
    uint8_t mainControlCmd;				//主要控制参数
    ShootMode shootMode;				//子弹发射模式
    DistinguishMode distinguishMode;	//识别模式
    BulletType bulletType;				//弹丸种类
    EnemyColor enemyType;				//敌人颜色
    uint8_t shootSpeed;					//子弹射速
    CarType carType;					//车辆类型
    BuffBias buffBias;					//神符偏差
} Stm32CmdStruct;
```

我们自定的通讯协议一共有 18 个字节

| Byte0      | Byte1      | Byte2      | Byte3      | Byte4      | Byte5      | Byte6      | Byte7      | Byte8 |
| :--------- | :--------- | :--------- | :--------- | :--------- | :--------- | :--------- | :--------- | :---- |
| 0xD4       | length     | CRC8       | cmd1       | carType    | speed      | pitch      | pitch      | pitch |
| **Byte9** | **Byte10** | **Byte11** | **Byte12** | **Byte13** | **Byte14**         | **Byte15**       | **Byte16** | **Byte17** |
| pitch         | yaw            | yaw            | yaw            | yaw           | CNTR            | CNTR        | CRC16  | CRC16      |

> * 0xD4：帧头 
> * length：接收数据长度
> * CNTR：惯导UKF计数位
> * CRC8：参与CRC8校验
> * cmd1：包含识别模式(3)，敌人颜色(1)，神符偏差(4)
> * carType：车辆类型
> * speed：子弹射速
> * pitch：惯导UKF解算pitch轴数据
> * yaw：惯导UKF解算yaw轴数据
> * CRC16：参与CRC16校验

### 3.数据发送结构体
```cpp
typedef struct{
    Point3FUnionStruct armorCoordinate;        //坐标帧
    Point3FUnionStruct armorCoordinateReal; //绝对坐标帧
    FormatTrans32Struct pitchData;                   //云台pitch轴角度
    FormatTrans32Struct yawData;                     //云台yaw轴角度
    FormatTrans32Struct barrelToArmor;           //枪管到装甲板的距离（弃用）
    bool getOrderFlag;		          //获得命令Flag
    bool captureCmd;                                           //是否接收到32端发来的数据Flag
    bool sameTargetFlag;                                     //同一目标Flag
    FormatTrans32Struct distinguishTime;         //识别模式
    FormatTrans16Struct feedbackCNTR;           //CRC校验码
} TX2CmdStruct;
```

我们自定的通讯协议一共有 16 个字节

| Byte0      | Byte1      | Byte2      | Byte3      | Byte4      | Byte5      | Byte6      | Byte7      |
| :--------- | :--------- | :--------- | :--------- | :--------- | :--------- | :--------- | :--------- |
| 0xD4       | length     | CRC8       | cmd        |   yaw      |   yaw        | yaw         | yaw           |
| **Byte8** | **Byte9** | **Byte10** | **Byte11** | **Byte12** | **Byte13** | **Byte14** | **Byte15** |
| pitch        | pitch         | pitch          | pitch         | CNTR         | CNTR        | CRC16       | CRC16      |
> * 0xAD：帧头 
> * cmd：控制Falg：包括同一目标Flag、锁定目标Flag，工作状态Flag
> * CRC：参与CRC校验
> * pitch：视觉牛顿解方程解算pitch轴数据
> * yaw：视觉牛顿解方程解算yaw轴数据

## 参与贡献

| 功能 | 负责人 |  联系方式|
| ------ | ------ | ------ |
| 识别装甲板 | 谢祥平  | WeChat:13330642377
| 装甲板预判打击 | 李承蒙  | qq:1059505781 WeChat:wxid_psnjdfzd0w6f12
| 能量机关识别 | 高万禄  | qq:2209120827 WeChat:WanluGao
| 能量机关预判 代码架构 | 梁睿哲  | qq:1473742892 WeChat:lrz1473742892
| 识别特定ID | 郗哲  | qq:397891352 WeChat:XZZ9650
| 反小陀螺 | 陈远斐  | qq:2298408812 WeChat:vx15077358740

 特别鸣谢：
   *识别部分：何煜
   *坐标系转化及数据分析部分：蔡承霖
   *代码架构：李云灏
   *追踪器与分类器：江超
   *能量机关：张丁介
