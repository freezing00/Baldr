#ifndef _RM_DEFINE_H_
#define _RM_DEFINE_H_

#include <opencv2/opencv.hpp>

using namespace std;

//������ɫ
typedef enum {
    ENEMY_RED = 0,
    ENEMY_BLUE
} EnemyColor;
//�ҷ���ɫ
typedef enum {
    OWN_RED = 0,
    OWN_BLUE
} OwnColor;
//��������ģʽ
typedef enum {
    SAVE_DISABLE = 0,
    AUTO_SAVE
} SaveSampleMode;
enum EXTREMUM {
    SHORT_SIDE = 0,
    LONG_SIDE
};
//ʶ��ģʽ
typedef enum {
    TX2_STOP = 0,                           //ֹͣ
    TX2_DISTINGUISH_ARMOR,                  //װ�װ�
    TX2_DISTINGUISH_BUFF,                   //С��
    TX2_DISTINGUISH_BIG_BUFF,               //���
} DistinguishMode;
//���ƫ��
typedef enum {
    NO_CHANGE = 0,
    UP = 1,
    DOWN = 2,
    FRONT = 4,
    BACK = 8
} BuffBias;
//��������
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
//װ�����
typedef enum {
    ARMOR_SMALL = 0,
    ARMOR_BIG
} ArmorType;
//32λ������
typedef union {
    uchar u8_temp[4];
    float float_temp;
    int32_t s32_temp;
    uint32_t u32_temp;
} FormatTrans32Struct;
//16λ������
typedef union {
    uchar u8_temp[2];
    int16_t s16_temp;
    uint16_t u16_temp;
} FormatTrans16Struct;
//���ģʽ
typedef enum {
    MANUAL_SINGLE = 0,
    MANUAL_CONTINUOUS,
    AUTO_CONTINUOUS
} ShootMode;
//�ӵ�����
typedef enum {
    SMALL_BULLET = 0,
    BIG_BULLET
} BulletType;
//Point3f �ṹ��
typedef struct {
    FormatTrans16Struct x;
    FormatTrans16Struct y;
    FormatTrans16Struct z;
} Point3FUnionStruct;
//���������ṹ��
struct SampleDataStruct {
    cv::Mat image;
    bool classifyState = false;
};
#endif
