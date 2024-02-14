//
// Created by elsa on 24-2-14.
//

#ifndef ANSWER_ANSWER_NODE_H
#define ANSWER_ANSWER_NODE_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Answer : public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr clickPointPublisher;

    cv::Mat image;
    std::vector<cv::Vec4i> lines;
    cv::Point matchLocation;
    int mode;
    double angle;

    void clickPointLoc_callback();
    void image_callback(const sensor_msgs::msg::Image &msg);

    void line_detector();//获取判定线坐标
    void note_detector();//识别音符坐标（模板匹配
    void image_threshold();//对图片进行二值化处理

public:
    Answer();
};

#endif //ANSWER_ANSWER_NODE_H
