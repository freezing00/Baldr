#ifndef _NET_H_
#define _NET_H_

#include "tool/Conf.h"

namespace bp {
    class Net {
    public:
        Net() {}
        ~Net() {}
        /**
        * Ԥ������
        * @param input
        */
        int predict_one(cv::Mat &input);
        /**
        * ����xmlģ��
        * @param filename
        */
        void load(const std::string &filename);
        /**
        * Ԥ��ͼ���Ԥ����
        * @param input
        */
        static cv::Mat bpImgPreProcess(cv::Mat input);
        std::vector<int> layerNeuronNum;
        std::string activeFunction = "tanh";
        float learningRate;
    private:
        int _resultNumber;
        void forward();
        /**
        * ��ʼ��bpģ�Ͳ���
        * @layer_neuron_num_
        */
        void initNet(std::vector<int> layer_neuron_num_);
    protected:
        std::vector<cv::Mat> layer;
        std::vector<cv::Mat> weights;
        std::vector<cv::Mat> bias;
        cv::Mat activationFunction(cv::Mat &x, const std::string &func_type);
        static cv::Mat sigmoid(cv::Mat &x);
        static cv::Mat tanh(cv::Mat &x);
    };
}
#endif
