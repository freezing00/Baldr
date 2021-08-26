#include "armor/bpPredict/net.h"

#define MINIMUM_CONFIDENCE 0.95f
namespace bp {
    cv::Mat Net::activationFunction(cv::Mat &x, const std::string &func_type) {
        activeFunction = func_type;
        cv::Mat fx;
        if (func_type == "sigmoid")   //0-1  多用于二分类
        {
            fx = sigmoid(x);
        }
        if (func_type == "tanh") {     //-1 - 1  在多分类上比sigmoid效果好
            fx = tanh(x);
        }
        return fx;
    }
    cv::Mat Net::sigmoid(cv::Mat &x) {
        cv::Mat exp_x, fx;
        cv::exp(-x, exp_x);
        fx = 1.0 / (1.0 + exp_x);
        return fx;
    }
    cv::Mat Net::tanh(cv::Mat &x) {
        cv::Mat exp_x_, exp_x, fx;
        cv::exp(-x, exp_x_);
        cv::exp(x, exp_x);
        fx = (exp_x - exp_x_) / (exp_x + exp_x_);
        return fx;
    }
    void Net::initNet(std::vector<int> layer_neuron_num_) {
        layerNeuronNum = layer_neuron_num_;
        layer.resize(layerNeuronNum.size());
        for (int i = 0; i < layer.size(); i++) {
            layer[i].create(layerNeuronNum[i], 1, CV_32FC1);
        }
        LOG::info("Generate layers, successfully!");
        weights.resize(layer.size() - 1);
        bias.resize(layer.size() - 1);
        for (int i = 0; i < (layer.size() - 1); ++i) {
            weights[i].create(layer[(i + 1)].rows, layer[i].rows, CV_32FC1);
            bias[i] = cv::Mat::zeros(layer[(i + 1)].rows, 1, CV_32FC1);
        }
        LOG::info("Generate weights matrices and bias, successfully!");
        LOG::info("Initialise Net, done!");
    }
    void Net::forward() {
        for (int i = 0; i < layerNeuronNum.size() - 1; ++i) {
            cv::Mat product = weights[i] * layer[i] + bias[i];
            layer[(i + 1)] = activationFunction(product, activeFunction);
        }
    }
    int Net::predict_one(cv::Mat &input) {
        if (input.empty()) {
            return 0;
        }
        if (input.rows == (layer[0].rows) && input.cols == 1) {
            double confidence = 0.0;
            layer[0] = input;
            forward();
            cv::Mat layer_out = layer[layer.size() - 1];
            cv::Point predict_maxLoc;
            minMaxLoc(layer_out, NULL, &confidence, NULL, &predict_maxLoc, cv::noArray());
            _resultNumber = predict_maxLoc.y;
           // std::cout<<_resultNumber<<std::endl;
            if (confidence > MINIMUM_CONFIDENCE && _resultNumber != 7) {
                return (_resultNumber + 1);
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    }
    void Net::load(const std::string &filename) {
        cv::FileStorage fs;
        fs.open(filename, cv::FileStorage::READ);
        fs["layer_neuron_num"] >> layerNeuronNum;
        initNet(layerNeuronNum);
        for (int i = 0; i < weights.size(); i++) {
            std::string weight_name = "weight_" + std::to_string(i);
            fs[weight_name] >> weights[i];
        }
        for (int i = 0; i < bias.size(); i++) {
            std::string bias_name = "bias_" + std::to_string(i);
            fs[bias_name] >> bias[i];
        }
        fs["learning_rate"] >> learningRate;
        fs["activation_function"] >> activeFunction;
        fs.release();
    }
    cv::Mat Net::bpImgPreProcess(cv::Mat input) {
        cv::Mat predictImg = cv::Mat(400, 1, CV_32FC1);
        cv::threshold(input, input, 0, 255, cv::THRESH_OTSU);
        input.convertTo(input, CV_32FC1);
        float pixs[400] = {0};
        for (int i = 0; i < input.rows; i++) {
            for (int j = 0; j < input.cols; j++) {
                float temp = (input.at<float>(i, j) / 255.0f);
                input.at<float>(i, j) = temp;
            }
        }
        int counts = 0;
        for (int i = 0; i < input.rows; i++) {
            for (int j = 0; j < input.cols; j++) {
                pixs[counts] = input.at<float>(i, j);
                counts++;
            }
        }
        for (int i = 0; i < 400; i++) {
            predictImg.at<float>(i, 0) = pixs[i];
        }
        return predictImg;
    }
}

