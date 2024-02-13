#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Answer : public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr clickPointPublisher;
    rclcpp::TimerBase::SharedPtr clickTimer;

    cv::Mat image;
    std::vector<cv::Vec4i> lines;
    cv::Point matchLocation;

    void clickPointLoc_callback(){
            auto message = geometry_msgs::msg::Point32();
            message.x = matchLocation.x + 50;
            message.y = 323;
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Send position: (" << message.x << " " << message.y << ")");
            clickPointPublisher->publish(message);
    }

    void image_callback(const sensor_msgs::msg::Image &msg) {
        cv_bridge::CvImagePtr cvImage;
        cvImage = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8);
        cvImage->image.copyTo(image); //获取ROS传来的图片

        line_detector();//获取判定线坐标
        note_detector();//识别音符坐标（模板匹配）
        if(abs( matchLocation.y - 323) <= 120){//发送模拟点击坐标
            clickPointLoc_callback();
        }
    }

    //获取判定线坐标
    void line_detector(){
        cv::Mat color_dst,dst;
        cv::cvtColor( image, color_dst, cv::COLOR_BGR2GRAY);
        cv::Canny( color_dst, dst, 100, 200);
        cv::HoughLinesP( dst, lines, 1, CV_PI/180, 10, 500, 22);

//        for( cv::Vec< int, 4> n : lines)
//            std::cout << n;
//        std::cout << "\n"; //测试用
    }

    //识别音符坐标（模板匹配）
    void note_detector(){
        cv::Mat tmp = cv::imread("click.png");
        cv::resize( tmp, tmp, cv::Size( 150, 15));
        int result_rows = image.rows - tmp.rows + 1;
        int result_cols = image.cols - tmp.cols + 1;
        cv::Mat result( result_cols, result_rows, CV_32FC1);
        cv::matchTemplate( image, tmp, result, cv::TM_CCOEFF_NORMED);
        cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

        double minValue,maxValue;
        cv::Point minLocation,maxLocation;
        cv::minMaxLoc( result, &minValue, &maxValue, &minLocation, &maxLocation, cv::Mat());
        matchLocation = maxLocation;

        //std::cout << "matchLocation: " << matchLocation << std::endl; //测试用
    }

public:
    Answer() : Node("answer_node") {
        imageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
                "/raw_image", 10, std::bind( &Answer::image_callback, this, _1));
        clickPointPublisher = this->create_publisher<geometry_msgs::msg::Point32>( "/click_position", 10);
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Answer>());
    rclcpp::shutdown();
    return 0;
}
