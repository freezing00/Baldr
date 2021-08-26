#ifndef _LOG_H_
#define _LOG_H_

#include <fstream>
#include "opencv2/opencv.hpp"

class LOG {
private:
    bool logLevel = true;
    std::ofstream stream;
    double saveTime = (double) cv::getTickCount();

    /**
     * 单例对象
     */
    static LOG &instance() {
        static LOG logger;
        return logger;
    }

    /**
     * 获得日志时间戳
     * @return 时间戳
     */
    static std::string getTime() {
#ifdef _MSC_VER
        time_t time_seconds = time(0);
        static struct tm ltm;
        localtime_s(&ltm, &time_seconds);
        return tostring(ltm.tm_hour) + ":" + tostring(ltm.tm_min) + ":" + tostring(ltm.tm_sec);
#else
        time_t now = time(0);
        tm *ltm = localtime(&now);
        return tostring(ltm->tm_hour) + ":" + tostring(ltm->tm_min) + ":" + tostring(ltm->tm_sec);
#endif
    }

    /**
     * 获得保存图片的时间戳
     * @return 时间戳
     */
    static std::string getSaveImgTime() {
#ifdef _MSC_VER
        time_t time_seconds = time(0);
        struct tm ltm;
        localtime_s(&ltm, &time_seconds);
        return tostring(1 + ltm.tm_mon) + "_" + tostring(ltm.tm_mday) + "_" + tostring(ltm.tm_hour) + "_" + tostring(ltm.tm_min) + "_" + tostring(ltm.tm_sec);
#else
        time_t now = time(0);
        tm *ltm = localtime(&now);
        return tostring(1 + ltm->tm_mon) + "_" + tostring(ltm->tm_mday) + "_" + tostring(ltm->tm_hour) + "_" + tostring(ltm->tm_min) + "_" + tostring(ltm->tm_sec);
#endif
    }

    /**
     * 判断上一次和这一次时间间隔是否大于1秒
     * @return
     */
    static bool timeDelay() {
        double time = ((double) cv::getTickCount() - instance().saveTime) / cv::getTickFrequency();
        return time > 1.0f;
    }

public:

    ~LOG() {
        stream.close();
    }

    /**
     * 设置日志存储位置和保存图片位置
     * @param logPath 日志存储位置
     * @param saveImgPath 保存图片位置
     */
    static void setDestination(const std::string &logPath) {
        std::cout << "Save log to : " << logPath << std::endl;

        //打开日志存储文件
        instance().stream.open(logPath, std::ios::out);
#ifdef _MSC_VER
        time_t time_seconds = time(0);
        struct tm ltm;
        localtime_s(&ltm, &time_seconds);
        info("Now time : " + tostring(1900 + ltm.tm_year) + "." + tostring(1 + ltm.tm_mon) + "." + tostring(ltm.tm_mday) + "  " + tostring(ltm.tm_hour) + ":" + tostring(ltm.tm_min) + ":" + tostring(ltm.tm_sec));
#else
        time_t now = time(0);
        tm *ltm = localtime(&now);
        info("Now time : " + tostring(1900 + ltm->tm_year) + "." + tostring(1 + ltm->tm_mon) + "." + tostring(ltm->tm_mday) + "  " + tostring(ltm->tm_hour) + ":" + tostring(ltm->tm_min));
#endif
    }

    /**
     * 设置日志级别，缺省为info，可选debug
     * @param level 日志级别
     */
    static void setLevel(const char *level) {
        if (level == "debug") {
            instance().logLevel = false;
        } else {
            instance().logLevel = true;
        }
    }

    /**
     * 关闭日志
     */
    static void close() {
        instance().stream.close();
    }

    /**
     * 打印info日志
     * @param log 日志内容
     */
    static void info(const char *log) {
        std::string _log = "[INFO " + getTime() + "]: ";
        _log.append(log).append("\n");
        instance().stream << _log;
        std::cout << _log;
    }

    /**
     * 打印info日志
     * @param log 日志内容
     */
    static void info(const std::string &log) {
        LOG::info(log.c_str());
    }

    /**
     * 打印debug日志
     * @param log 日志内容
     */
    static void debug(const char *log) {
        if (!instance().logLevel) {
            std::string _log = "[DEBUG " + getTime() + "]: ";
            _log.append(log).append("\n");
            instance().stream << _log;
            std::cout << _log;
        }
    }

    /**
     * 打印debug日志
     * @param log 日志内容
     */
    static void debug(const std::string &log) {
        LOG::debug(log.c_str());
    }

    /**
     * 打印error日志
     * @param log 日志内容
     */
    static void error(const char *log) {
        std::string _log = "[ERROR " + getTime() + "]: ";
        _log.append(log).append("\n");
        instance().stream << _log;
        std::cout << _log;
    }

    /**
     * 打印error日志
     * @param log 日志内容
     */
    static void error(const std::string &log) {
        LOG::error(log.c_str());
    }

    static std::string tostring(int i) {
        return std::to_string(i);
    }

    static std::string tostring(double i) {
        return std::to_string(i);
    }

    static std::string tostring(long i) {
        return std::to_string(i);
    }

    static std::string tostring(float i) {
        return std::to_string(i);
    }

    static std::string tostring(unsigned i) {
        return std::to_string(i);
    }

    static std::string tostring(bool i) {
        return std::to_string(i);
    }

    static std::string tostring(const char *i) {
        return std::string(i);
    }
};

#endif
