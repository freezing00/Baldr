#ifndef _POINT_UTIL_H_
#define _POINT_UTIL_H_

#include <vector>
#include "RMDefine.h"

//������
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

    //ȷ���������򲻻�Խ��
    static inline bool makeRectSafe(cv::Rect &rect, const cv::Size &size) {
        if (rect.x < 0)                                //rect.x����С��0����㲻��С��0��
            rect.x = 0;
        if (rect.x + rect.width > size.width)          //rect.x+rect.width���ܴ���src.size.width��x+width�õ�ROI�Ŀ���յ㣬���ܴ���ԴͼƬ��ȣ�
            rect.width = size.width - rect.x;
        if (rect.y < 0)                                //rect.y����С��0����㲻��С��0
            rect.y = 0;
        if (rect.y + rect.height > size.height)        //rect.y+rect.height���ܴ���src.size.height��y+height�õ�ROI�ĸ߶��յ㣬���ܴ���ԴͼƬ�߶ȣ�
            rect.height = size.height - rect.y;
        return !(rect.width <= 0 || rect.height <= 0);
    }

    /**
     * �ҳ����εļ�ֵ��
     * @param rect ����
     * @param flag true�ǳ��ߣ�false�Ƕ̱�
     * @return
     */
    static inline float findExtremumOfSide(cv::RotatedRect &rect, bool flag) {
        if (flag)
            return rect.size.height > rect.size.width ? rect.size.height : rect.size.width;
        else
            return rect.size.height < rect.size.width ? rect.size.height : rect.size.width;
    }

    /**
     * ��һ����̬�������ҵ������С�ľ���
     * @param rects ���ζ�̬����
     * @return �����С�����±�
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
     * ��һ����̬�������ҵ�������С���±�
     * @param arr ��̬����
     * @return �±�
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
     * �������������غϺ���Ƕ�ĵ�
     * @param rectangle ����1
     * @param partRect ����2
     * @return ��Ƕ����
     */
    static inline uint8_t doesRectangleContainPoint(cv::RotatedRect &rectangle, cv::RotatedRect &partRect) {

        //ת��Ϊ����
        cv::Point2f corners[4];
        rectangle.points(corners);
        cv::Point2f *lastItemPointer = (corners + sizeof(corners) / sizeof(corners[0]));
        std::vector<cv::Point2f> contour(corners, lastItemPointer);
        //�ж�,��measureDist����Ϊfalseʱ��������ֵΪ+1����ʾ���������ڲ�������ֵΪ-1����ʾ�������ⲿ������ֵΪ0����ʾ��������
        uint8_t inside = 0;
        cv::Point2f partVertices[4];

        partRect.points(partVertices);
        //partVertices[4] = partRect.center;
        for (size_t i = 0; i < 4; i++) {
            if (pointPolygonTest(contour, partVertices[i], false) >= 0)        //���ڵ���0���������ڲ���������
                inside++;
        }
        return inside;
    }

    /**
     * ��һ�Ե����ϳ�һ��װ�װ�
     * @param rect_1 ����1
     * @param rect_2 ����2
     * @return �ϳɵ�װ�װ�
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
     * ��ת���εĳ����
     * @param rect ��ת����
     * @return �����
     */
    static inline float getRectLengthWidthRatio(cv::RotatedRect &rect) {
        float longSide = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;   //��ȡʶ������γ�
        float shortSide = rect.size.height < rect.size.width ? rect.size.height : rect.size.width;  //��ȡʶ������ο�
        return longSide / shortSide;
    }

    /**
     * ����ֱ�ߵļн�
     * @return �н�
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
     * �����pt1,��pt2,���pt0���γɵļнǵĽǶ�
     * @param pt1 ��1
     * @param pt2 ��2
     * @param pt0 ��0
     * @return �нǽǶ�
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
     * ����Ŀ�����
     * @param mat ͼƬ
     * @param rect �����ľ���
     * @return ͼƬ
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
     * �������β���ʾ
     * @param mat ͼƬ
     * @param rect �����ľ���
     * @param name ��ʾ����
     */
    static inline void drawRectShow(cv::Mat mat, const cv::RotatedRect &rect, const char *name = "test") {
        mat = drawRect(mat, rect);
        cv::imshow(name, mat);
        cv::waitKey(1);
    }

    /**
     * ��ʾ�����ľ���
     * @param mat ͼƬ
     * @param name ��ʾ����
     */
    static inline void showRect(cv::Mat mat, std::string name) {
        cv::namedWindow(name, 0);
        cv::imshow(name, mat);
        cv::waitKey(1);
    }

    /**
     * �������μ�����ʾ
     * @param mat ͼƬ
     * @param rects ���μ�
     * @param name ��ʾ����
     */
    static inline void drawRectsShow(cv::Mat mat, const std::vector<cv::RotatedRect> &rects, const char *name = "testRects") {
        for (int i = 0; i < rects.size(); i++) {
            drawRect(mat, rects[i]);
        }
        showRect(mat, name);
    }

    /**
     * ���㷽��
     * @param arr �㼯
     * @return ����
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
     * �����˳������¼�
     * @param sig
     */
    static void exitHandler(int sig) {
        LOG::info("Interrupt by keyboard!");
        LOG::close();
        exit(-1);
    }

    /**
     * �������
     * @param p1 ��1
     * @param p2 ��2
     * @return ����
     */
    static inline float pointDistance(const cv::Point2f &p1, const cv::Point2f &p2) {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    /**
     * �������
     * @param p1 ��1
     * @param p2 ��2
     * @return ����
     */
    static inline double pointDistance(const cv::Point2d &p1, const cv::Point2d &p2) {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    /**
     * �������
     * @param p1 ��1
     * @param p2 ��2
     * @return ����
     */
    static inline double pointDistance(const cv::Point &p1, const cv::Point &p2) {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    /**
     * �ռ���һ�㵽ԭ��ľ���
     * @param p ��
     * @return ����
     */
    static inline float Distance3DToZero(const cv::Point3f &p) {
        return sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    /**
     * �Ƕ�ת����
     * @param p �Ƕ�ֵ
     * @return ����ֵ
     */
    static inline double angleToRadian(double p) {
        return p * PI() / 180.0f;
    }

    /**
     * ����ת�Ƕ�
     * @param p ����ֵ
     * @return �Ƕ�ֵ
     */
    static inline float radianToAngle(float p) {
        return p * 180.0f / PI_F();
    }

    /**
     * ��������G
     * @return G
     */
    static inline float GRAVITY() {
        return 9.78634f;
    }

    /**
     * Բ����
     */
    static inline float PI_F() {
        return float(CV_PI);
    }

    /**
     * Բ����
     */
    static inline double PI() {
        return CV_PI;
    }

    /**
     * 3��3���������
     */
    static inline cv::Mat structuringElement3() {
        return instance().StructuringElement3;
    }

    /**
     * 5��5���������
     */
    static inline cv::Mat structuringElement5() {
        return instance().StructuringElement5;
    }

    /**
     * 7��7���������
     */
    static inline cv::Mat structuringElement7() {
        return instance().StructuringElement7;
    }
};

#endif
