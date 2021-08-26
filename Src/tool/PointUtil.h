#ifndef _POINT_UTIL_H_
#define _POINT_UTIL_H_

#include <vector>
#include "RMDefine.h"

//工具类
class Util {
private:
    static Util &instance() {
        static Util util;
        return util;
    }

    const cv::Mat StructuringElement3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    const cv::Mat StructuringElement5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    const cv::Mat StructuringElement7 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

public:

    //确保矩形区域不会越界
    static inline bool makeRectSafe(cv::Rect &rect, const cv::Size &size) {
        if (rect.x < 0)                                //rect.x不能小于0（起点不能小于0）
            rect.x = 0;
        if (rect.x + rect.width > size.width)          //rect.x+rect.width不能大于src.size.width（x+width得到ROI的宽度终点，不能大于源图片宽度）
            rect.width = size.width - rect.x;
        if (rect.y < 0)                                //rect.y不能小于0（起点不能小于0
            rect.y = 0;
        if (rect.y + rect.height > size.height)        //rect.y+rect.height不能大于src.size.height（y+height得到ROI的高度终点，不能大于源图片高度）
            rect.height = size.height - rect.y;
        return !(rect.width <= 0 || rect.height <= 0);
    }

    /**
     * 找出矩形的极值边
     * @param rect 矩形
     * @param flag true是长边，false是短边
     * @return
     */
    static inline float findExtremumOfSide(cv::RotatedRect &rect, bool flag) {
        if (flag)
            return rect.size.height > rect.size.width ? rect.size.height : rect.size.width;
        else
            return rect.size.height < rect.size.width ? rect.size.height : rect.size.width;
    }

    /**
     * 从一个动态数组中找到面积最小的矩形
     * @param rects 矩形动态数组
     * @return 面积最小矩形下标
     */
    static inline int findMaxAreaRect(std::vector<cv::RotatedRect> rects) {
        int j = 0;
        float max = rects[0].size.area();
        for (size_t i = 1; i < rects.size(); i++) {
            if (rects[i].size.area() > max) {
                max = rects[i].size.area();
                j = (int) i;
            }
        }
        return j;
    }

    /**
     * 从一个动态数组中找到距离最小的下标
     * @param arr 动态数组
     * @return 下标
     */
    static inline int findMinDistance(std::vector<float> &arr) {
        int j = 0;
        float min = arr[0];
        for (size_t i = 1; i < arr.size(); i++) {
            if (arr[i] < min) {
                min = arr[i];
                j = (int) i;
            }
        }
        return j;
    }

    /**
     * 计算两个矩形重合和内嵌的点
     * @param rectangle 矩形1
     * @param partRect 矩形2
     * @return 内嵌数量
     */
    static inline uint8_t doesRectangleContainPoint(cv::RotatedRect &rectangle, cv::RotatedRect &partRect) {

        //转化为轮廓
        cv::Point2f corners[4];
        rectangle.points(corners);
        cv::Point2f *lastItemPointer = (corners + sizeof(corners) / sizeof(corners[0]));
        std::vector<cv::Point2f> contour(corners, lastItemPointer);
        //判断,当measureDist设置为false时，若返回值为+1，表示点在轮廓内部，返回值为-1，表示在轮廓外部，返回值为0，表示在轮廓上
        uint8_t inside = 0;
        cv::Point2f partVertices[4];

        partRect.points(partVertices);
        //partVertices[4] = partRect.center;
        for (size_t i = 0; i < 4; i++) {
            if (pointPolygonTest(contour, partVertices[i], false) >= 0)        //大于等于0则在轮廓内部或轮廓上
                inside++;
        }
        return inside;
    }

    /**
     * 用一对灯条合成一个装甲板
     * @param rect_1 灯条1
     * @param rect_2 灯条2
     * @return 合成的装甲板
     */
    static inline cv::RotatedRect boundingRRect(const cv::RotatedRect &rect_1, const cv::RotatedRect &rect_2) {
        const cv::Point2f &pl = rect_1.center, &pr = rect_2.center;
        cv::Point2f center = (pl + pr) / 2.0;
        cv::RotatedRect wh_1 = rect_1;
        cv::RotatedRect wh_2 = rect_2;
        float long_1 = Util::findExtremumOfSide(wh_1, LONG_SIDE);
        float long_2 = Util::findExtremumOfSide(wh_2, LONG_SIDE);
        float width = Util::pointDistance(pl, pr);
        float height = (long_2 + long_1) / (float) 2.0;
        float angle = std::atan2(rect_2.center.y - rect_1.center.y, rect_2.center.x - rect_1.center.x);
        return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / PI_F());
    }

    /**
     * 旋转矩形的长款比
     * @param rect 旋转矩形
     * @return 长宽比
     */
    static inline float getRectLengthWidthRatio(cv::RotatedRect &rect) {
        float longSide = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;   //获取识别出矩形长
        float shortSide = rect.size.height < rect.size.width ? rect.size.height : rect.size.width;  //获取识别出矩形宽
        return longSide / shortSide;
    }

    /**
     * 两条直线的夹角
     * @return 夹角
     */
    static inline float lineToLineAngle(cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &p3, cv::Point2f &p4) {
        if (p2.x == p1.x) {
            p2.x += 1e-10f;
        }
        if (p3.x == p4.x) {
            p3.x += 1e-10f;
        }
        float tan1 = (p2.y - p1.y) / (p2.x - p1.x);
        float tan2 = (p4.y - p3.y) / (p4.x - p3.x);
        float angle1 = radianToAngle(atanf(tan1));
        float angle2 = radianToAngle(atanf(tan2));
        float skew = fabs(fabs(angle1 - angle2) - 90);
        return 90.0f - skew;
    }

    /**
     * 计算点pt1,点pt2,与点pt0所形成的夹角的角度
     * @param pt1 点1
     * @param pt2 点2
     * @param pt0 点0
     * @return 夹角角度
     */
    static inline float getAngle(cv::Point2f pt1, cv::Point2f pt2, cv::Point2f pt0) {
        float dx1 = pt1.x - pt0.x;
        float dy1 = pt1.y - pt0.y;
        float dx2 = pt2.x - pt0.x;
        float dy2 = pt2.y - pt0.y;
        float angle_line = (dx1 * dx2 + dy1 * dy2) / sqrtf((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10f);
        return acosf(angle_line) * 180.0f / 3.141592653f;
    }

    /**
     * 画出目标矩形
     * @param mat 图片
     * @param rect 待画的矩形
     * @return 图片
     */
    static inline cv::Mat drawRect(cv::Mat &mat, const cv::RotatedRect &rect) {
        cv::Point2f verts[4];
        rect.points(verts);
        for (int i = 0; i < 4; i++) {
            cv::line(mat, verts[i], verts[(i + 1) % 4], cv::Scalar(100, 200, 255), 1);
        }
        return mat;
    }

    /**
     * 画出矩形并显示
     * @param mat 图片
     * @param rect 待画的矩形
     * @param name 显示名称
     */
    static inline void drawRectShow(cv::Mat mat, const cv::RotatedRect &rect, const char *name = "test") {
        mat = drawRect(mat, rect);
        cv::imshow(name, mat);
        cv::waitKey(1);
    }

    /**
     * 显示画出的矩形
     * @param mat 图片
     * @param name 显示名称
     */
    static inline void showRect(cv::Mat mat, std::string name) {
        cv::namedWindow(name, 0);
        cv::imshow(name, mat);
        cv::waitKey(1);
    }

    /**
     * 画出矩形集并显示
     * @param mat 图片
     * @param rects 矩形集
     * @param name 显示名称
     */
    static inline void drawRectsShow(cv::Mat mat, const std::vector<cv::RotatedRect> &rects, const char *name = "testRects") {
        for (int i = 0; i < rects.size(); i++) {
            drawRect(mat, rects[i]);
        }
        showRect(mat, name);
    }

    /**
     * 计算方差
     * @param arr 点集
     * @return 方差
     */
    static inline float varianceDataCalculate(std::vector<float> &arr) {
        float averageValue = 0.0f;
        float variance = 0.0f;
        for (size_t i = 1; i < arr.size(); i++) {
            averageValue += arr[i];

        }
        averageValue = averageValue / arr.size();

        for (size_t i = 1; i < arr.size(); i++) {
            variance += (arr[i] - averageValue) * (arr[i] - averageValue);
        }
        variance = variance / arr.size();
        variance = sqrtf(variance);

        return variance;
    }

    /**
     * 处理退出代码事件
     * @param sig
     */
    static void exitHandler(int sig) {
        LOG::info("Interrupt by keyboard!");
        LOG::close();
        exit(-1);
    }

    /**
     * 两点距离
     * @param p1 点1
     * @param p2 点2
     * @return 距离
     */
    static inline float pointDistance(const cv::Point2f &p1, const cv::Point2f &p2) {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    /**
     * 两点距离
     * @param p1 点1
     * @param p2 点2
     * @return 距离
     */
    static inline double pointDistance(const cv::Point2d &p1, const cv::Point2d &p2) {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    /**
     * 两点距离
     * @param p1 点1
     * @param p2 点2
     * @return 距离
     */
    static inline double pointDistance(const cv::Point &p1, const cv::Point &p2) {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    /**
     * 空间中一点到原点的距离
     * @param p 点
     * @return 距离
     */
    static inline float Distance3DToZero(const cv::Point3f &p) {
        return sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    /**
     * 角度转弧度
     * @param p 角度值
     * @return 弧度值
     */
    static inline double angleToRadian(double p) {
        return p * PI() / 180.0f;
    }

    /**
     * 弧度转角度
     * @param p 弧度值
     * @return 角度值
     */
    static inline float radianToAngle(float p) {
        return p * 180.0f / PI_F();
    }

    /**
     * 引力常数G
     * @return G
     */
    static inline float GRAVITY() {
        return 9.78634f;
    }

    /**
     * 圆周率
     */
    static inline float PI_F() {
        return float(CV_PI);
    }

    /**
     * 圆周率
     */
    static inline double PI() {
        return CV_PI;
    }

    /**
     * 3×3矩阵运算核
     */
    static inline cv::Mat structuringElement3() {
        return instance().StructuringElement3;
    }

    /**
     * 5×5矩阵运算核
     */
    static inline cv::Mat structuringElement5() {
        return instance().StructuringElement5;
    }

    /**
     * 7×7矩阵运算核
     */
    static inline cv::Mat structuringElement7() {
        return instance().StructuringElement7;
    }
};

#endif
