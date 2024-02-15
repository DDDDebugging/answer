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
    cv::Mat tmp = cv::imread("click.png");
    std::vector<cv::Vec4i> lines;
    std::vector<cv::Vec4i> turned_lines;
    cv::Point matchLocation;
    cv::Point turned_matchLocation;
    int mode;
    double angle;
    double pro;

    void clickPointLoc_callback();
    void image_callback(const sensor_msgs::msg::Image &msg);

    void line_detector();//获取判定线坐标
    void turned_line_detector(cv::Mat & src_dst);//获取旋转后判定线坐标
    void note_detector();//识别音符坐标（模板匹配

public:
    Answer();
};

#endif //ANSWER_ANSWER_NODE_H
