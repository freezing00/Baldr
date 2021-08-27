#ifndef _CONF_H_
#define _CONF_H_

#include "tool/fileOperation.h"
#include "tool/PointUtil.h"

//装甲板真实长度参数
struct ArmorRealData {
    float length = 120.0f;
    float width = 55.0f;
};
//装甲板识别参数
struct ArmorPara {
    EnemyColor enemyColor;
    int minLightBarArea = 20;
    int minLightBarLength = 8;
    int minLightBarWidth = 2;
    int maxLightBarLength = 200;
    int maxLightBarWidth = 50;
    float maxErrorAngle = 15;
    int grayThreshold_PURPLE = 150;
    int grayThreshold_RED = 100;
    int separationThreshold_RED = 80;
    int grayThreshold_BLUE = 160;
    int separationThreshold_BLUE = 80;
    int separationThreshold_GREEN = 10;
};

//神符识别参数
struct BuffPara {
    int minLightBarGray_RED_R = 220;
    int minLightBarGray_RED_G = 100;
    int minLightBarGray_BLUE_R = 30;
    int minLightBarGray_BLUE_G = 150;
    int minLightBarSize = 15;
    int maxLightBarSize = 200;
    int grayThreshold_RED = 60;                            //灰度二值化阈值-红色
    int grayThreshold_BLUE = 100;                            //灰度二值化阈值-蓝色
    int separationThreshold_RED = 40;                    //色彩分离二值化阈值-红色
    int separationThreshold_BLUE = 100;                    //色彩分离二值化阈值-蓝色
};
//神符圆参数
struct ParaCircle {
    float buffPredictAngle = 25.0f;
    float buffRadBias = 2.0f;
    float realCantileverLength = 730.0f;
    float armorAngleBias = 5.0f;
    float minToArmorDistance = 220;//80;
    float maxToArmorDistance = 600;
    float minArea = 80.0f;
    float maxArea = 400;
    float minLengthWidthRatio = 0.7f;
    float maxLengthWidthRatio = 1.5f;
};

//新版神符参数
struct NewBuffPara {
    float grayThresholdValue = 60;
    float channelThresholdValue = 60;
    int singleIterations = 1;
    int binMatIterations = 2;
    float armorSideRatio_MAX = 4.2f;
    float armorSideRatio_MIN = 1.4f;
    float armorTo_R_SizeRatio_MAX = 9;
    float armorTo_R_SizeRatio_MIN = 1.5f;
    float armorTo_R_DistanceRatio_MAX = 11.0f;
    float armorTo_R_DistanceRatio_MIN = 6.0f;
    float longSideRatioForROI = 2.0f;
    float shortSideRatioForROI = 0.8f;
    float targetSizeRadio = 0.8f;
};

//圆心参数
struct R_Para {
    float squreSideRatio = 1.5f;
    float squreSizeLimit_MAX = 160;
    float squreSizeLimit_MIN = 12;
};

struct BuffPredict {

};

//相机参数
struct CameraConfiguration {
    double gainValue = 3;                       //增益
    int exposureValue = 5000;                   //曝光时间
    double gammaValue = 1;                      //gamma
};

//安装参数
struct InstallData {
    cv::Point3f PTZToCameraBias;                //云台中心到相机中心三方向上的偏差
    cv::Point3f PTZToBarrelBias;                //云台中心到炮管出弹口中心三方向上的偏差
    cv::Point3f BarrelToCameraBias;                //出弹口中心到云台中心三方向上的偏差
};

class ArmorDataFactory {
private:
    static ArmorDataFactory &instance() {
        static ArmorDataFactory armorDataFactory;
        return armorDataFactory;
    }

    static void initArmorData() {
        //小装甲
        instance().smallArmor.length = 130.0f;
        instance().smallArmor.width = 55.0f;
        //大装甲
        instance().bigArmor.length = 225.0f;
        instance().bigArmor.width = 55.0f;
        //神符
        instance().buffArmor.length = 245.0f;
        instance().buffArmor.width = 137.0f;
        //圆心
        instance().r_Armor.length = 60.0f;
        instance().r_Armor.width = 60.0f;
    }
public:
    static ArmorRealData getSmallArmor() {
        return instance().smallArmor;
    }

    static ArmorRealData getBigArmor() {
        return instance().bigArmor;
    }

    static ArmorRealData getBuffArmor() {
        return instance().buffArmor;
    }

    static ArmorRealData get_R_Armor() {
        return instance().r_Armor;
    }


    static void resetAllConfig() {
        initArmorData();
    }

    static void readArmor(const std::string &filename) {
        initArmorData();
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeArmor(filename);
            LOG::error("default config not find, write config to " + filename);
        } else {
            cv::FileNode map_node = fs["SmallArmor"];
            map_node["length"] >> instance().smallArmor.length;
            map_node["width"] >> instance().smallArmor.width;
            LOG::debug("smallArmor.length= " + std::to_string(instance().smallArmor.length));
            LOG::debug("smallArmor.width= " + std::to_string(instance().smallArmor.width));

            map_node = fs["BigArmor"];
            map_node["length"] >> instance().bigArmor.length;
            map_node["width"] >> instance().bigArmor.width;
            LOG::debug("bigArmor.length= " + std::to_string(instance().bigArmor.length));
            LOG::debug("bigArmor.width= " + std::to_string(instance().bigArmor.width));

            map_node = fs["BuffArmor"];
            map_node["length"] >> instance().buffArmor.length;
            map_node["width"] >> instance().buffArmor.width;
            LOG::debug("buffArmor.length= " + std::to_string(instance().buffArmor.length));
            LOG::debug("buffArmor.width= " + std::to_string(instance().buffArmor.width));

            map_node = fs["R_Armor"];
            map_node["length"] >> instance().r_Armor.length;
            map_node["width"] >> instance().r_Armor.width;
            LOG::debug("r_Armor.length= " + std::to_string(instance().r_Armor.length));
            LOG::debug("r_Armor.width= " + std::to_string(instance().r_Armor.width));

        }
    }

    static void writeArmor(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //写入注释
        fs.writeComment("装甲板实际大小数据\n");
        //cv::cvWriteComment(*fs, "装甲板实际大小数据\n", 0);

        //写入ARMOR节点
        fs.writeComment("小装甲板数据");
        fs << "SmallArmor" << "{"
           << "length" << instance().smallArmor.length
           << "width" << instance().smallArmor.width
           << "}";

        fs.writeComment("大装甲板数据");
        fs << "BigArmor" << "{"
           << "length" << instance().bigArmor.length
           << "width" << instance().bigArmor.width
           << "}";

        fs.writeComment("神符装甲板数据");
        fs << "BuffArmor" << "{"
           << "length" << instance().buffArmor.length
           << "width" << instance().buffArmor.width
           << "}";

        fs.writeComment("圆心R数据");
        fs << "R_Armor" << "{"
            << "length" << instance().r_Armor.length
            << "width" << instance().r_Armor.width
            << "}";

        //销毁
        fs.release();
    }

public:
    ArmorRealData smallArmor;
    ArmorRealData bigArmor;
    ArmorRealData buffArmor;
    ArmorRealData r_Armor;
};

class ArmorParaFactory {
private:
    static ArmorParaFactory &instance() {
        static ArmorParaFactory armorParaFactory;
        return armorParaFactory;
    }

public:
    static ArmorPara getArmorPara() {
        return instance().armorPara;
    }

    static void resetAllConfig() {
        instance().armorPara = ArmorPara();
    }

    static void readArmorPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeArmorPara(filename);
            LOG::error("default config not find, write config to " + filename);
        } else {
            // 读入普通节点
            //读入map类型的节点
            cv::FileNode map_node = fs["armorPara"];
            map_node["minLightBarArea"] >> instance().armorPara.minLightBarArea;
            map_node["minLightBarLength"] >> instance().armorPara.minLightBarLength;
            map_node["minLightBarWidth"] >> instance().armorPara.minLightBarWidth;
            map_node["maxLightBarLength"] >> instance().armorPara.maxLightBarLength;
            map_node["maxLightBarWidth"] >> instance().armorPara.maxLightBarWidth;
            map_node["maxErrorAngle"] >> instance().armorPara.maxErrorAngle;
            map_node["grayThreshold_PURPLE"] >> instance().armorPara.grayThreshold_PURPLE;
            map_node["grayThreshold_RED"] >> instance().armorPara.grayThreshold_RED;
            map_node["separationThreshold_RED"] >> instance().armorPara.separationThreshold_RED;
            map_node["grayThreshold_BLUE"] >> instance().armorPara.grayThreshold_BLUE;
            map_node["separationThreshold_BLUE"] >> instance().armorPara.separationThreshold_BLUE;
            map_node["separationThreshold_GREEN"] >> instance().armorPara.separationThreshold_GREEN;
            LOG::debug("armorPara.minLightBarArea=" + std::to_string(instance().armorPara.minLightBarArea));
            LOG::debug("armorPara.minLightBarLength=" + std::to_string(instance().armorPara.minLightBarLength));
            LOG::debug("armorPara.minLightBarWidth=" + std::to_string(instance().armorPara.minLightBarWidth));
            LOG::debug("armorPara.maxLightBarLength=" + std::to_string(instance().armorPara.maxLightBarLength));
            LOG::debug("armorPara.maxLightBarWidth=" + std::to_string(instance().armorPara.maxLightBarWidth));
            LOG::debug("armorPara.maxErrorAngle=" + std::to_string(instance().armorPara.maxErrorAngle));
            LOG::debug("armorPara.grayThreshold_PURPLE=" + std::to_string(instance().armorPara.grayThreshold_PURPLE));
            LOG::debug("armorPara.grayThreshold_RED=" + std::to_string(instance().armorPara.grayThreshold_RED));
            LOG::debug("armorPara.separationThreshold_RED=" + std::to_string(instance().armorPara.separationThreshold_RED));
            LOG::debug("armorPara.grayThreshold_BLUE=" + std::to_string(instance().armorPara.grayThreshold_BLUE));
            LOG::debug("armorPara.separationThreshold_BLUE=" + std::to_string(instance().armorPara.separationThreshold_BLUE));
            LOG::debug("armorPara.separationThreshold_GREEN=" + std::to_string(instance().armorPara.separationThreshold_GREEN));
        }
    }

    static void writeArmorPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);

        fs.writeComment("装甲板识别数据");
        fs << "armorPara" << "{";
        fs.writeComment("最小的灯条面积");
        fs << "minLightBarArea" << instance().armorPara.minLightBarArea;
        fs.writeComment("最小的灯条长度");
        fs << "minLightBarLength" << instance().armorPara.minLightBarLength;
        fs.writeComment("最小的灯条宽度");
        fs << "minLightBarWidth" << instance().armorPara.minLightBarWidth;
        fs.writeComment("最大灯条长度");
        fs << "maxLightBarLength" << instance().armorPara.maxLightBarLength;
        fs.writeComment("最大灯条宽度");
        fs << "maxLightBarWidth" << instance().armorPara.maxLightBarWidth;
        fs.writeComment("容许的最大角度差");
        fs << "maxErrorAngle" << instance().armorPara.maxErrorAngle;
        fs.writeComment("灰度二值化阈值-紫色");
        fs << "grayThreshold_PURPLE" << instance().armorPara.grayThreshold_PURPLE;
        fs.writeComment("灰度二值化阈值-红色");
        fs << "grayThreshold_RED" << instance().armorPara.grayThreshold_RED;
        fs.writeComment("色彩分离二值化阈值-红色");
        fs << "separationThreshold_RED" << instance().armorPara.separationThreshold_RED;
        fs.writeComment("灰度二值化阈值-蓝色");
        fs << "grayThreshold_BLUE" << instance().armorPara.grayThreshold_BLUE;
        fs.writeComment("色彩分离二值化阈值-蓝色");
        fs << "separationThreshold_BLUE" << instance().armorPara.separationThreshold_BLUE;
        fs.writeComment("色彩分离二值化阈值-绿色");
        fs << "separationThreshold_GREEN" << instance().armorPara.separationThreshold_GREEN;
        fs << "}";
        //销毁
        fs.release();
    }

public:
    ArmorPara armorPara;
};

class BuffParaFactory {
private:
    static BuffParaFactory &instance() {
        static BuffParaFactory buffParaFactory;
        return buffParaFactory;
    }

public:
    static BuffPara getPara() {
        return instance().buffPara;
    }

    static ParaCircle getParaCircle() {
        return instance().paraCircle;
    }

    static void resetAllConfig() {
        instance().buffPara = BuffPara();
        instance().paraCircle = ParaCircle();
    }

    static void readBuffPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeBuffPara(filename);
            LOG::error("default config not find, write config to " + filename);
        } else {
            //读入map类型的节点
            cv::FileNode map_node = fs["BuffPara"];
            map_node["minLightBarGray_RED_R"] >> instance().buffPara.minLightBarGray_RED_R;
            map_node["minLightBarGray_RED_G"] >> instance().buffPara.minLightBarGray_RED_G;
            map_node["minLightBarGray_BLUE_R"] >> instance().buffPara.minLightBarGray_BLUE_R;
            map_node["minLightBarGray_BLUE_G"] >> instance().buffPara.minLightBarGray_BLUE_G;
            map_node["minLightBarSize"] >> instance().buffPara.minLightBarSize;
            map_node["maxLightBarSize"] >> instance().buffPara.maxLightBarSize;
            map_node["grayThreshold_RED"] >> instance().buffPara.grayThreshold_RED;
            map_node["grayThreshold_BLUE"] >> instance().buffPara.grayThreshold_BLUE;
            map_node["separationThreshold_RED"] >> instance().buffPara.separationThreshold_RED;
            map_node["separationThreshold_BLUE"] >> instance().buffPara.separationThreshold_BLUE;
            LOG::debug("BuffPara.minLightBarGray_RED_R=" + std::to_string(instance().buffPara.minLightBarGray_RED_R));
            LOG::debug("BuffPara.minLightBarGray_RED_G=" + std::to_string(instance().buffPara.minLightBarGray_RED_G));
            LOG::debug("BuffPara.minLightBarGray_BLUE_R=" + std::to_string(instance().buffPara.minLightBarGray_BLUE_R));
            LOG::debug("BuffPara.minLightBarGray_BLUE_G=" + std::to_string(instance().buffPara.minLightBarGray_BLUE_G));
            LOG::debug("BuffPara.grayThreshold_RED=" + std::to_string(instance().buffPara.grayThreshold_RED));
            LOG::debug("BuffPara.grayThreshold_BLUE=" + std::to_string(instance().buffPara.grayThreshold_BLUE));
            LOG::debug("BuffPara.separationThreshold_RED=" + std::to_string(instance().buffPara.separationThreshold_RED));
            LOG::debug("BuffPara.separationThreshold_BLUE=" + std::to_string(instance().buffPara.separationThreshold_BLUE));
            LOG::debug("BuffPara.minLightBarSize=" + std::to_string(instance().buffPara.minLightBarSize));
            LOG::debug("BuffPara.maxLightBarSize=" + std::to_string(instance().buffPara.maxLightBarSize));

            map_node = fs["paraCircle"];
            map_node["armorAngleBias"] >> instance().paraCircle.armorAngleBias;
            map_node["minToArmorDistance"] >> instance().paraCircle.minToArmorDistance;
            map_node["maxToArmorDistance"] >> instance().paraCircle.maxToArmorDistance;
            map_node["minArea"] >> instance().paraCircle.minArea;
            map_node["maxArea"] >> instance().paraCircle.maxArea;
            map_node["minLengthWidthRatio"] >> instance().paraCircle.minLengthWidthRatio;
            map_node["maxLengthWidthRatio"] >> instance().paraCircle.maxLengthWidthRatio;
            map_node["buffRadBias"] >> instance().paraCircle.buffRadBias;
            map_node["buffPredictAngle"] >> instance().paraCircle.buffPredictAngle;
            map_node["realCantileverLength"] >> instance().paraCircle.realCantileverLength;
            LOG::debug("paraCircle.armorAngleBias=" + std::to_string(instance().paraCircle.armorAngleBias));
            LOG::debug("paraCircle.minToArmorDistance=" + std::to_string(instance().paraCircle.minToArmorDistance));
            LOG::debug("paraCircle.maxToArmorDistance=" + std::to_string(instance().paraCircle.maxToArmorDistance));
            LOG::debug("paraCircle.minArea=" + std::to_string(instance().paraCircle.minArea));
            LOG::debug("paraCircle.maxArea=" + std::to_string(instance().paraCircle.maxArea));
            LOG::debug("paraCircle.minLengthWidthRatio=" + std::to_string(instance().paraCircle.minLengthWidthRatio));
            LOG::debug("paraCircle.maxLengthWidthRatio=" + std::to_string(instance().paraCircle.maxLengthWidthRatio));
            LOG::debug("paraCircle.buffRadBias=" + std::to_string(instance().paraCircle.buffRadBias));
            LOG::debug("paraCircle.buffPredictAngle=" + std::to_string(instance().paraCircle.buffPredictAngle));
            LOG::debug("paraCircle.realCantileverLength=" + std::to_string(instance().paraCircle.realCantileverLength));
        }
    }

    static void writeBuffPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //写入注释
        fs.writeComment("神符数据");
        //写入para节点
        fs << "BuffPara" << "{";
        fs.writeComment("RED板灯red  channel最小灰度值");
        fs << "minLightBarGray_RED_R" << instance().buffPara.minLightBarGray_RED_R;
        fs.writeComment("RED板灯Green channel最小灰度值");
        fs << "minLightBarGray_RED_G" << instance().buffPara.minLightBarGray_RED_G;
        fs.writeComment("BLUE板灯red channel最小灰度值");
        fs << "minLightBarGray_BLUE_R" << instance().buffPara.minLightBarGray_BLUE_R;
        fs.writeComment("BLUE板灯Green channel最小灰度值");
        fs << "minLightBarGray_BLUE_G" << instance().buffPara.minLightBarGray_BLUE_G;
        fs.writeComment("最小轮廓尺寸");
        fs << "minLightBarSize" << instance().buffPara.minLightBarSize;
        fs.writeComment("最大轮廓尺寸");
        fs << "maxLightBarSize" << instance().buffPara.maxLightBarSize;
        fs.writeComment("红色灰度二值化阈值");
        fs << "grayThreshold_RED" << instance().buffPara.grayThreshold_RED;
        fs.writeComment("蓝色灰度二值化阈值");
        fs << "grayThreshold_BLUE" << instance().buffPara.grayThreshold_BLUE;
        fs.writeComment("红色色彩分离二值化阈值");
        fs << "separationThreshold_RED" << instance().buffPara.separationThreshold_RED;
        fs.writeComment("蓝色色彩分离二值化阈值");
        fs << "separationThreshold_BLUE" << instance().buffPara.separationThreshold_BLUE;
        fs << "}";
        //写入paraCircle节点
        fs << "paraCircle" << "{";
        fs.writeComment("圆心角度");
        fs << "armorAngleBias" << instance().paraCircle.armorAngleBias;
        fs.writeComment("最小距离");
        fs << "minToArmorDistance" << instance().paraCircle.minToArmorDistance;
        fs.writeComment("最大距离");
        fs << "maxToArmorDistance" << instance().paraCircle.maxToArmorDistance;
        fs.writeComment("最小面积");
        fs << "minArea" << instance().paraCircle.minArea;
        fs.writeComment("最大面积");
        fs << "maxArea" << instance().paraCircle.maxArea;
        fs.writeComment("最小长宽比");
        fs << "minLengthWidthRatio" << instance().paraCircle.minLengthWidthRatio;
        fs.writeComment("最大长款此");
        fs << "maxLengthWidthRatio" << instance().paraCircle.maxLengthWidthRatio;
        fs.writeComment("神符预判角度");
        fs << "buffPredictAngle" << instance().paraCircle.buffPredictAngle;
        fs.writeComment("神符偏差");
        fs << "buffRadBias" << instance().paraCircle.buffRadBias;
        fs.writeComment("悬臂真实长度");
        fs << "realCantileverLength" << instance().paraCircle.realCantileverLength;
        fs << "}";
        //销毁
        fs.release();
    }

public:
    BuffPara buffPara;
    ParaCircle paraCircle;
};

class NewBuffParaFactory {
private:
    static NewBuffParaFactory& instance() {
        static NewBuffParaFactory newBuffParaFactory;
        return newBuffParaFactory;
    }
public:
    static NewBuffPara getNewBuff() {
        return instance().newBuffPara;
    }

    static R_Para get_R() {
        return instance().r_Para;
    }

    static void resetAllConfig() {
        instance().newBuffPara = NewBuffPara();
        instance().r_Para = R_Para();
    }

    static void readNewBuffPara(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeNewBuffPara(filename);
            LOG::error("default config not find, write config to " + filename);
        }
        else {
            //读入map类型节点
            cv::FileNode buffNode = fs["NewBuffPara"];
            buffNode["grayThresholdValue"] >> instance().newBuffPara.grayThresholdValue;
            buffNode["channelThresholdValue"] >> instance().newBuffPara.channelThresholdValue;
            buffNode["singleIterations"] >> instance().newBuffPara.singleIterations;
            buffNode["binMatIterations"] >> instance().newBuffPara.binMatIterations;
            buffNode["armorSideRatio_MAX"] >> instance().newBuffPara.armorSideRatio_MAX;
            buffNode["armorSideRatio_MIN"] >> instance().newBuffPara.armorSideRatio_MIN;
            buffNode["armorTo_R_SizeRatio_MAX"] >> instance().newBuffPara.armorTo_R_SizeRatio_MAX;
            buffNode["armorTo_R_SizeRatio_MIN"] >> instance().newBuffPara.armorTo_R_SizeRatio_MIN;
            buffNode["armorTo_R_DistanceRatio_MAX"] >> instance().newBuffPara.armorTo_R_DistanceRatio_MAX;
            buffNode["armorTo_R_DistanceRatio_MIN"] >> instance().newBuffPara.armorTo_R_DistanceRatio_MIN;
            buffNode["longSideRatioForROI"] >> instance().newBuffPara.longSideRatioForROI;
            buffNode["shortSideRatioForROI"] >> instance().newBuffPara.shortSideRatioForROI;
            buffNode["targetSizeRadio"] >> instance().newBuffPara.targetSizeRadio;
            LOG::debug("NewBuffPara.grayThresholdValue_R=" + std::to_string(instance().newBuffPara.grayThresholdValue));
            LOG::debug("NewBuffPara.channelThresholdValue_R=" + std::to_string(instance().newBuffPara.channelThresholdValue));
            LOG::debug("NewBuffPara.singleIterations=" + std::to_string(instance().newBuffPara.singleIterations));
            LOG::debug("NewBuffPara.binMatIterations=" + std::to_string(instance().newBuffPara.binMatIterations));
            LOG::debug("NewBuffPara.armorSideRatio_MAX=" + std::to_string(instance().newBuffPara.armorSideRatio_MAX));
            LOG::debug("NewBuffPara.armorSideRatio_MIN=" + std::to_string(instance().newBuffPara.armorSideRatio_MIN));
            LOG::debug("NewBuffPara.armorTo_R_SizeRatio_MAX=" + std::to_string(instance().newBuffPara.armorTo_R_SizeRatio_MAX));
            LOG::debug("NewBuffPara.armorTo_R_SizeRatio_MIN=" + std::to_string(instance().newBuffPara.armorTo_R_SizeRatio_MIN));
            LOG::debug("NewBuffPara.armorTo_R_DistanceRatio_MAX=" + std::to_string(instance().newBuffPara.armorTo_R_DistanceRatio_MAX));
            LOG::debug("NewBuffPara.armorTo_R_DistanceRatio_MIN=" + std::to_string(instance().newBuffPara.armorTo_R_DistanceRatio_MIN));
            LOG::debug("NewBuffPara.longSideRatioForROI=" + std::to_string(instance().newBuffPara.longSideRatioForROI));
            LOG::debug("NewBuffPara.shortSideRatioForROI=" + std::to_string(instance().newBuffPara.shortSideRatioForROI));
            LOG::debug("NewBuffPara.targetSizeRadio=" + std::to_string(instance().newBuffPara.targetSizeRadio));


            cv::FileNode R_Node = fs["R_Para"];
            R_Node["squreSideRatio"] >> instance().r_Para.squreSideRatio;
            R_Node["squreSizeLimit_MAX"] >> instance().r_Para.squreSizeLimit_MAX;
            R_Node["squreSizeLimit_MIN"] >> instance().r_Para.squreSizeLimit_MIN;
            LOG::debug("R_Para.squreSideRatio=" + std::to_string(instance().r_Para.squreSideRatio));
            LOG::debug("R_Para.squreSizeLimit_MAX=" + std::to_string(instance().r_Para.squreSizeLimit_MAX));
            LOG::debug("R_Para.squreSizeLimit_MIN=" + std::to_string(instance().r_Para.squreSizeLimit_MIN));

        }
    }
    static void writeNewBuffPara(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //写入注释
        fs.writeComment("新版神符数据");
        //写入节点
        fs << "NewBuffPara" << "{";
        fs.writeComment("灰度图阈值");
        fs << "grayThresholdValue" << instance().newBuffPara.grayThresholdValue;
        fs.writeComment("通道图阈值");
        fs << "channelThresholdValue" << instance().newBuffPara.channelThresholdValue;
        fs.writeComment("单通道图膨胀迭代次数");
        fs << "singleIterations" << instance().newBuffPara.singleIterations;
        fs.writeComment("二值图膨胀迭代次数");
        fs << "binMatIterations" << instance().newBuffPara.binMatIterations;
        fs.writeComment("装甲板长宽比上限");
        fs << "armorSideRatio_MAX" << instance().newBuffPara.armorSideRatio_MAX;
        fs.writeComment("装甲板长宽比下限");
        fs << "armorSideRatio_MIN" << instance().newBuffPara.armorSideRatio_MIN;
        fs.writeComment("装甲板边长上限");
        fs << "armorTo_R_SizeRatio_MAX" << instance().newBuffPara.armorTo_R_SizeRatio_MAX;
        fs.writeComment("装甲板边长下限");
        fs << "armorTo_R_SizeRatio_MIN" << instance().newBuffPara.armorTo_R_SizeRatio_MIN;
        fs.writeComment("装甲板与R中心距离上限");
        fs << "armorTo_R_DistanceRatio_MAX" << instance().newBuffPara.armorTo_R_DistanceRatio_MAX;
        fs.writeComment("装甲板与R中心距离下限");
        fs << "armorTo_R_DistanceRatio_MIN" << instance().newBuffPara.armorTo_R_DistanceRatio_MIN;
        fs.writeComment("长边ROI比例");
        fs << "longSideRatioForROI" << instance().newBuffPara.longSideRatioForROI;
        fs.writeComment("短边ROI比例");
        fs << "shortSideRatioForROI" << instance().newBuffPara.shortSideRatioForROI;
        fs.writeComment("目标装甲板缩小比例");
        fs << "targetSizeRadio" << instance().newBuffPara.targetSizeRadio;
        fs << "}";

        fs << "R_Para" << "{";
        fs.writeComment("类正方形长宽比限制");
        fs << "squreSideRatio" << instance().r_Para.squreSideRatio;
        fs.writeComment("方形轮廓面积上限");
        fs << "squreSizeLimit_MAX" << instance().r_Para.squreSizeLimit_MAX;
        fs.writeComment("方形轮廓面积下限");
        fs << "squreSizeLimit_MIN" << instance().r_Para.squreSizeLimit_MIN;
        fs << "}";
        //销毁
        fs.release();
    }
public:
    NewBuffPara newBuffPara;
    R_Para r_Para;
};

class CameraConfigurationFactory {
private:
    static CameraConfigurationFactory &instance() {
        static CameraConfigurationFactory cameraConfigurationFactory;
        return cameraConfigurationFactory;
    }

    /**
     * 向配置文件中写入一个相机配置节点
     * @param cameraConfiguration 待写入的节点
     * @param name 节点名称
     * @param comment 待写入的注释
     * @param fs 配置文件指针
     */
    static void writeCameraConfigurationStruct(CameraConfiguration &cameraConfiguration, const char *name, const char *comment, cv::FileStorage &fs) {
        fs.writeComment("\n");
        fs.writeComment(comment);
        fs << name << "{"
           << "gainValue" << cameraConfiguration.gainValue
           << "exposureValue" << cameraConfiguration.exposureValue
           << "gammaValue" << cameraConfiguration.gammaValue
           << "}";
    }

    /**
     * 从配置文件中读入一个相机配置节点
     * @param cameraConfiguration 待读入的节点
     * @param name 节点名称
     * @param fs 配置文件指针
     */
    static void readCameraConfigurationStruct(CameraConfiguration &cameraConfiguration, const char *name, cv::FileStorage &fs) {
        cv::FileNode node = fs[name];
        node["gammaValue"] >> cameraConfiguration.gammaValue;
        node["exposureValue"] >> cameraConfiguration.exposureValue;
        node["gainValue"] >> cameraConfiguration.gainValue;
        LOG::debug(LOG::tostring(name) + ".gammaValue=" + std::to_string(cameraConfiguration.gammaValue));
        LOG::debug(LOG::tostring(name) + ".exposureValue=" + std::to_string(cameraConfiguration.exposureValue));
        LOG::debug(LOG::tostring(name) + ".gainValue=" + std::to_string(cameraConfiguration.gainValue));
    }

public:
    static CameraConfiguration getArmorCameraConfiguration() {
        return instance().armorCameraConfiguration;
    }

    static CameraConfiguration getBuffCameraConfiguration() {
        return instance().buffCameraConfiguration;
    }

    static void resetAllConfig() {
        instance().armorCameraConfiguration = CameraConfiguration();
        instance().buffCameraConfiguration = CameraConfiguration();
    }

    static void readCameraConfiguration(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeCameraConfiguration(filename);
            LOG::error("default config not find, write config to " + filename);
        } else {
            readCameraConfigurationStruct(instance().armorCameraConfiguration, "ArmorCameraConfiguration", fs);
            readCameraConfigurationStruct(instance().buffCameraConfiguration, "BuffCameraConfiguration", fs);
        }
        fs.release();
    }

    static void writeCameraConfiguration(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //写入注释
        fs.writeComment("相机设置");
        writeCameraConfigurationStruct(instance().armorCameraConfiguration, "ArmorCameraConfiguration", "识别装甲板相机参数", fs);
        writeCameraConfigurationStruct(instance().buffCameraConfiguration, "BuffCameraConfiguration", "识别神符相机参数", fs);
        //销毁
        fs.release();
    }

public:

    CameraConfiguration armorCameraConfiguration;
    CameraConfiguration buffCameraConfiguration;
};

class CodeSet {
private:
    bool useDebugGuiFlag = false;
    bool takePictureFlag = false;
    int grayAverage = 20;
    bool useAutoSaveSampleMode = 0;  //0:停止保存 1:自动保存    默认:0

    static CodeSet &instance() {
        static CodeSet codeSet;
        return codeSet;
    }

public:
    static void resetAllConfig() {
        instance() = CodeSet();
    }

    static void readCodeSet(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeCodeSet(filename);
            LOG::error("default config not find, write config to " + filename);
        } else {
            fs["takePictureFlag"] >> instance().takePictureFlag;
            fs["useDebugGuiFlag"] >> instance().useDebugGuiFlag;
            fs["grayAverage"] >> instance().grayAverage;
            fs["useAutoSaveSampleMode"] >> instance().useAutoSaveSampleMode;

            LOG::debug("codeSet.takePictureFlag=" + std::to_string(instance().takePictureFlag));
            LOG::debug("codeSet.useDebugGuiFlag=" + std::to_string(instance().useDebugGuiFlag));
            LOG::debug("codeSet.grayAverage=" + std::to_string(instance().grayAverage));
            LOG::debug("codeSet.useAutoSaveSampleMode=" + std::to_string(instance().useAutoSaveSampleMode));
        }
        fs.release();
    }

    static void writeCodeSet(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //写入注释
        fs.writeComment("代码设置\n");
        fs.writeComment("Linux下显示图像");
        fs << "useDebugGuiFlag" << instance().useDebugGuiFlag;
        fs.writeComment("代码运行时录像");
        fs << "takePictureFlag" << instance().takePictureFlag;
        fs.writeComment("自动曝光的平均灰度");
        fs << "grayAverage" << instance().grayAverage;
        fs.writeComment("0:停止保存 1:自动保存  默认:0");
        fs << "useAutoSaveSampleMode" << instance().useAutoSaveSampleMode;
        //销毁
        fs.release();
    }

    static bool getUseDebugGuiFlag() {
        return instance().useDebugGuiFlag;
    }

    static bool getTakePictureFlag() {
        return instance().takePictureFlag;
    }

    static int getGrayAverage() {
        return instance().grayAverage;
    }

    static SaveSampleMode getUseAutoSaveSampleMode() {
        return (SaveSampleMode) instance().useAutoSaveSampleMode;
    }
};

class InstallDataFactory {
private:
    static InstallDataFactory &instance() {
        static InstallDataFactory installDataFactory;
        return installDataFactory;
    }

    /**
     * 计算真实坐标信息
     * @param worldOriginInPZT 世界坐标在平行相机坐标系位置(云台中心为原点)
     * @param worldOriginInBarrel 世界坐标在平行相机坐标系位置(枪管出口为原点)
     * @param worldOriginInCamera 世界坐标在相机坐标系位置(相机为原点)
     * @param installData 安装参数
     */
    static void calculateRealCoordinate(cv::Point3f &worldOriginInPZT, cv::Point3f &worldOriginInBarrel, cv::Point3f &worldOriginInCamera, const InstallData &installData) {
        //获取装甲板对云台中心的位置(平行相机坐标系)
        worldOriginInPZT.z = worldOriginInCamera.z - installData.PTZToCameraBias.z;
        worldOriginInPZT.x = worldOriginInCamera.x - installData.PTZToCameraBias.x;
        worldOriginInPZT.y = worldOriginInCamera.y - installData.PTZToCameraBias.y;
        //获取装甲板对炮管出弹口中心的位置(平行相机坐标系)
        worldOriginInBarrel.z = worldOriginInCamera.z - installData.BarrelToCameraBias.z;
        worldOriginInBarrel.x = worldOriginInCamera.x - installData.BarrelToCameraBias.x;
        worldOriginInBarrel.y = worldOriginInCamera.y - installData.BarrelToCameraBias.y;
    }
    /**
     * 向配置文件中写入一个安装参数节点
     * @param installData 待写入的节点
     * @param name 节点名称
     * @param comment 待写入的注释
     * @param fs 配置文件指针
     */
    static void writeInstallDataStruct(InstallData &installData, const char *name, const char *comment, cv::FileStorage &fs) {
        fs.writeComment("\n");
        fs.writeComment(comment);
        fs << name << "{";
        fs.writeComment("云台中心到相机中心三方向上的偏差");
        fs << "PTZToCameraBias_x" << installData.PTZToCameraBias.x
           << "PTZToCameraBias_y" << installData.PTZToCameraBias.y
           << "PTZToCameraBias_z" << installData.PTZToCameraBias.z;
        fs.writeComment("云台中心到炮管出弹口中心三方向上的偏差");
        fs << "PTZToBarrelBias_x" << installData.PTZToBarrelBias.x
           << "PTZToBarrelBias_y" << installData.PTZToBarrelBias.y
           << "PTZToBarrelBias_z" << installData.PTZToBarrelBias.z
           << "}";
    }

    /**
     * 从配置文件中读入一个安装参数节点
     * @param installData 待读入的节点
     * @param name 节点名称
     * @param fs 配置文件指针
     */
    static void readInstallDataStruct(InstallData &installData, const char *name, cv::FileStorage &fs) {
        cv::FileNode node = fs[name];
        node["PTZToCameraBias_x"] >> installData.PTZToCameraBias.x;
        node["PTZToCameraBias_y"] >> installData.PTZToCameraBias.y;
        node["PTZToCameraBias_z"] >> installData.PTZToCameraBias.z;
        node["PTZToBarrelBias_x"] >> installData.PTZToBarrelBias.x;
        node["PTZToBarrelBias_y"] >> installData.PTZToBarrelBias.y;
        node["PTZToBarrelBias_z"] >> installData.PTZToBarrelBias.z;
        LOG::debug(LOG::tostring(name) + ".PTZToCameraBias.x=" + std::to_string(installData.PTZToCameraBias.x));
        LOG::debug(LOG::tostring(name) + ".PTZToCameraBias.y=" + std::to_string(installData.PTZToCameraBias.y));
        LOG::debug(LOG::tostring(name) + ".PTZToCameraBias.z=" + std::to_string(installData.PTZToCameraBias.z));
        LOG::debug(LOG::tostring(name) + ".PTZToBarrelBias.x=" + std::to_string(installData.PTZToBarrelBias.x));
        LOG::debug(LOG::tostring(name) + ".PTZToBarrelBias.y=" + std::to_string(installData.PTZToBarrelBias.y));
        LOG::debug(LOG::tostring(name) + ".PTZToBarrelBias.z=" + std::to_string(installData.PTZToBarrelBias.z));
        //获取出弹口中心到云台中心三方向上的偏差
        installData.BarrelToCameraBias.x = installData.PTZToCameraBias.x - installData.PTZToBarrelBias.x;
        installData.BarrelToCameraBias.y = installData.PTZToCameraBias.y - installData.PTZToBarrelBias.y;
        installData.BarrelToCameraBias.z = installData.PTZToCameraBias.z - installData.PTZToBarrelBias.z;
    }

    //初始化安装参数
    static void initInstallData() {
        //PS:安装信息是平行相机坐标系的安装信息
        //步兵
        instance().infantry.PTZToCameraBias.x = -56.5f;
        instance().infantry.PTZToCameraBias.y = 43.24f;
        instance().infantry.PTZToCameraBias.z = -98.2f;
        instance().infantry.PTZToBarrelBias.x = 0;
        instance().infantry.PTZToBarrelBias.y = 0;
        instance().infantry.PTZToBarrelBias.z = -66.6f;
        //老英雄
        instance().oldHero.PTZToCameraBias.x = 0;
        instance().oldHero.PTZToCameraBias.y = 48.5f;
        instance().oldHero.PTZToCameraBias.z = -182.51f;
        instance().oldHero.PTZToBarrelBias.x = 0;
        instance().oldHero.PTZToBarrelBias.y = 0;
        instance().oldHero.PTZToBarrelBias.z = -189.94f;
        //新英雄
        instance().newHero.PTZToCameraBias.x = -71.88f;
        instance().newHero.PTZToCameraBias.y = 69.5f;
        instance().newHero.PTZToCameraBias.z = -127.41f;
        instance().newHero.PTZToBarrelBias.x = 0;
        instance().newHero.PTZToBarrelBias.y = 0;
        instance().newHero.PTZToBarrelBias.z = -114.76f;
        //旧哨兵下云台
        instance().oldSentinelAbove.PTZToCameraBias.x = 0;
        instance().oldSentinelAbove.PTZToCameraBias.y = 41.4f;
        instance().oldSentinelAbove.PTZToCameraBias.z = 43.26f;
        instance().oldSentinelAbove.PTZToBarrelBias.x = 0;
        instance().oldSentinelAbove.PTZToBarrelBias.y = 0;
        instance().oldSentinelAbove.PTZToBarrelBias.z = -14.04f;
        //旧哨兵上云台
        instance().oldSentinelBelow.PTZToCameraBias.x = 0;
        instance().oldSentinelBelow.PTZToCameraBias.y = 41.4f;
        instance().oldSentinelBelow.PTZToCameraBias.z = 178.0f;
        instance().oldSentinelBelow.PTZToBarrelBias.x = 0;
        instance().oldSentinelBelow.PTZToBarrelBias.y = 0;
        instance().oldSentinelBelow.PTZToBarrelBias.z = 88.0f;
        //新哨兵下云台
        instance().newSentinelAbove.PTZToCameraBias.x = -56.5f;
        instance().newSentinelAbove.PTZToCameraBias.y = 43.50f;
        instance().newSentinelAbove.PTZToCameraBias.z = -38.46f;
        instance().newSentinelAbove.PTZToBarrelBias.x = 0;
        instance().newSentinelAbove.PTZToBarrelBias.y = 0;
        instance().newSentinelAbove.PTZToBarrelBias.z = -75.07f;
        //新哨兵上云台
        instance().newSentinelBelow.PTZToCameraBias.x = -56.50f;
        instance().newSentinelBelow.PTZToCameraBias.y = 43.50f;
        instance().newSentinelBelow.PTZToCameraBias.z = -123.60f;
        instance().newSentinelBelow.PTZToBarrelBias.x = 0;
        instance().newSentinelBelow.PTZToBarrelBias.y = 0;
        instance().newSentinelBelow.PTZToBarrelBias.z = -183.41f;
        //飞机
        instance().plane.PTZToCameraBias.x = 0;
        instance().plane.PTZToCameraBias.y = 65.3f;
        instance().plane.PTZToCameraBias.z = -119.65f;
        instance().plane.PTZToBarrelBias.x = 0;
        instance().plane.PTZToBarrelBias.y = 0.13f;
        instance().plane.PTZToBarrelBias.z = -78.00f;
    }

public:
    static void resetAllConfig() {
        instance().initInstallData();
    }

    //从配置文件读取安装参数
    static void readInstallData(const std::string &filename) {
        //初始化默认安装参数
        initInstallData();

        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeInstallData(filename);
            LOG::error("default config not find, write config to " + filename);
        } else {
            readInstallDataStruct(instance().infantry, "Infantry", fs);
            readInstallDataStruct(instance().oldHero, "OldHero", fs);
            readInstallDataStruct(instance().newHero, "NewHero", fs);
            readInstallDataStruct(instance().oldSentinelBelow, "OldSentinelBelow", fs);
            readInstallDataStruct(instance().oldSentinelAbove, "OldSentinelAbove", fs);
            readInstallDataStruct(instance().newSentinelBelow, "NewSentinelBelow", fs);
            readInstallDataStruct(instance().newSentinelAbove, "NewSentinelAbove", fs);
            readInstallDataStruct(instance().plane, "Plane", fs);
        }
        fs.release();
    }

    //写入安装参数到配置文件
    static void writeInstallData(const std::string &filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        writeInstallDataStruct(instance().infantry, "Infantry", "步兵安装参数", fs);
        writeInstallDataStruct(instance().oldHero, "OldHero", "老英雄安装参数", fs);
        writeInstallDataStruct(instance().newHero, "NewHero", "新英雄安装参数", fs);
        writeInstallDataStruct(instance().oldSentinelAbove, "OldSentinelBelow", "老哨兵下云台安装参数", fs);
        writeInstallDataStruct(instance().oldSentinelBelow, "OldSentinelAbove", "老哨兵上云台安装参数", fs);
        writeInstallDataStruct(instance().newSentinelAbove, "NewSentinelBelow", "新哨兵下云台安装参数", fs);
        writeInstallDataStruct(instance().newSentinelBelow, "NewSentinelAbove", "新哨兵上云台安装参数", fs);
        writeInstallDataStruct(instance().plane, "Plane", "飞机安装参数", fs);
        //销毁
        fs.release();

    }

    //根据车辆类型获取装甲板对于枪管出弹口的真实坐标信息
    static void getRealCoordinateDataByCarType(cv::Point3f &worldOriginInPZT, cv::Point3f &worldOriginInBarrel, cv::Point3f &worldOriginInCamera, CarType carType) {
        switch (carType) {
            case OLD_HERO:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().oldHero);
                break;
            case NEW_HERO:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().newHero);
                break;
            case NEW_SENTRY_BELOW:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().newSentinelBelow);
                break;
            case NEW_SENTRY_ABOVE:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().newSentinelAbove);
                break;
            case OLD_SENTRY_BELOW:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().oldSentinelBelow);
                break;
            case OLD_SENTRY_ABOVE:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().oldSentinelAbove);
                break;
            case PLANE:
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().plane);
                break;
            default :
                calculateRealCoordinate(worldOriginInPZT, worldOriginInBarrel, worldOriginInCamera, instance().infantry);
                break;
        }
    }

public:
    InstallData infantry;                //步兵
    InstallData oldHero;                //老英雄
    InstallData newHero;                //新英雄
    InstallData oldSentinelAbove;        //老哨兵上云台
    InstallData oldSentinelBelow;        //老哨兵下云台
    InstallData newSentinelAbove;        //新哨兵上云台
    InstallData newSentinelBelow;        //新哨兵下云台
    InstallData plane;                    //飞机
};

#endif // !_CONF_H_
