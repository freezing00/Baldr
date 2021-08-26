#ifndef _RM_DEFINE_H_
#define _RM_DEFINE_H_

#include <opencv2/opencv.hpp>

using namespace std;

//敌人颜色
typedef enum {
    ENEMY_RED = 0,
    ENEMY_BLUE
} EnemyColor;
//我方颜色
typedef enum {
    OWN_RED = 0,
    OWN_BLUE
} OwnColor;
//保存样本模式
typedef enum {
    SAVE_DISABLE = 0,
    AUTO_SAVE
} SaveSampleMode;
enum EXTREMUM {
    SHORT_SIDE = 0,
    LONG_SIDE
};
//识别模式
typedef enum {
    TX2_STOP = 0,                           //停止
    TX2_DISTINGUISH_ARMOR,                  //装甲板
    TX2_DISTINGUISH_BUFF,                   //小符
    TX2_DISTINGUISH_BIG_BUFF,               //大符
} DistinguishMode;
//神符偏差
typedef enum {
    NO_CHANGE = 0,
    UP = 1,
    DOWN = 2,
    FRONT = 4,
    BACK = 8
} BuffBias;
//车辆类型
typedef enum {
    INFANTRY = 0,
    OLD_HERO,
    NEW_HERO,
    OLD_SENTRY_BELOW,
    OLD_SENTRY_ABOVE,
    NEW_SENTRY_BELOW,
    NEW_SENTRY_ABOVE,
    PLANE
} CarType;
//装甲类别
typedef enum {
    ARMOR_SMALL = 0,
    ARMOR_BIG
} ArmorType;
//32位共用体
typedef union {
    uchar u8_temp[4];
    float float_temp;
    int32_t s32_temp;
    uint32_t u32_temp;
} FormatTrans32Struct;
//16位共用体
typedef union {
    uchar u8_temp[2];
    int16_t s16_temp;
    uint16_t u16_temp;
} FormatTrans16Struct;
//射击模式
typedef enum {
    MANUAL_SINGLE = 0,
    MANUAL_CONTINUOUS,
    AUTO_CONTINUOUS
} ShootMode;
//子弹类型
typedef enum {
    SMALL_BULLET = 0,
    BIG_BULLET
} BulletType;
//Point3f 结构体
typedef struct {
    FormatTrans16Struct x;
    FormatTrans16Struct y;
    FormatTrans16Struct z;
} Point3FUnionStruct;
//保存样本结构体
struct SampleDataStruct {
    cv::Mat image;
    bool classifyState = false;
};
#endif
