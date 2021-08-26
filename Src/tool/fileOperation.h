#ifndef _FILEOPERATION_H_
#define _FILEOPERATION_H_

#include "systemChoose.h"
#include "tool/RMLOG.h"

class FileOperation {
public:
    FileOperation() {}

    ~FileOperation() {}

    /**
     * 创建路径下的所有文件夹
     * @param folder 文件夹地址
     * @return 返回true
     */
    static bool createDirectory(const std::string &folder);

    /**
     * 获得目录下所有图片所组成的动态数组
     * @param path 路径
     * @param number 图片序号
     * @param pictureFormat 图片格式
     * @param imgList 返回的图片动态数组
     * @param readMode 读取模式
     */
    static void getImgFileInOrder(const std::string &path, uint32_t &number, const std::string &pictureFormat, std::vector<cv::Mat> &imgList, int readMode = 0);

    /**
     * 获取指定目录下的文件数量，从0开始扫描
     * @param path 文件路径
     * @param format 文件格式 如.avi .log .jpg .bmp
     * @return 文件数量
     */
    static uint32_t getFileSizeInOrder(const std::string &path, const std::string &format);

    /**
     * 保存图片
     * @param path 图片路径
     * @param number 图片编号-函数内部自增
     * @param img 要保存的图片
     * @param pictureFormat 图片格式
     */
    static void saveImg(const std::string &path, uint32_t &number, const cv::Mat &img, const std::string &pictureFormat);

    /**
     * 读取指定路径下所有文件名
     * @param path 文件夹地址
     * @param files 文件名动态数组
     * @param mode 读取模式 0:文件名包含子文件夹名 1:文件名不包含子文件夹名 2:文件名包含子文件夹内文件 默认: 1
     */
    static void getAllFilesName(std::string path, std::vector<std::string> &files, uint8_t mode = 1);
};

#endif
