#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

class ROS : public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr clickPointPublisher;

    static cv::Point clickPointLocation;
    std::vector<cv::Vec4i> lines;
    cv::Mat image;

    void image_callback(const sensor_msgs::msg::Image &msg) {
        RCLCPP_INFO(this->get_logger(), "Receive image");
        cv_bridge::CvImagePtr cvImage;
        cvImage = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8);
        cvImage->image.copyTo(image); //获取ROS传来的图片

        //接对图像的处理函数
        line_detector();
        music_point_detector();
        cv::setMouseCallback( "winter_homework_2024.x86_64", mouse_detector);

        auto data = geometry_msgs::msg::Point32();
        data.x = clickPointLocation.x;
        data.y = clickPointLocation.y;
        clickPointPublisher->publish(data);
    }

    void line_detector(){//确定直线位置
        cv::Mat dst;
        cv::Canny( image, dst, 50, 200, 3);
        cv::HoughLinesP( dst, lines, 1, CV_PI/180, 10, 200, 22);

        std::cout << lines[0][1] << ';' <<lines[0][3] << std::endl; //测试用
    }

    void music_point_detector(){//模板匹配找音符位置
        cv::Mat tmp = cv::imread("click.png");
        cv::cvtColor( image, image, cv::COLOR_BGR2GRAY);
        cv::cvtColor( tmp, tmp, cv::COLOR_BGR2GRAY);
        cv::resize( tmp, tmp, cv::Size( 50, 10));
        int result_rows = image.rows - tmp.rows + 1;
        int result_cols = image.cols - tmp.cols + 1;
        cv::Mat result( result_cols, result_rows, CV_32FC1);
        cv::matchTemplate( image, tmp, result, cv::TM_CCOEFF_NORMED);
        cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        double minValue,maxValue;
        cv::Point minLocation,maxLocation,matchLocation;
        cv::minMaxLoc( result, &minValue, &maxValue, &minLocation, &maxLocation, cv::Mat());
        matchLocation = maxLocation;

        std::cout << matchLocation <<std::endl;//测试用
    }

    static void mouse_detector( int event, int x, int y, int flags, void *userdata){ //获取鼠标点击坐标
        if(event == cv::EVENT_LBUTTONUP){
            clickPointLocation.x = x;
            clickPointLocation.y = y;

            std::cout << "x: " << clickPointLocation.x << "y: " << clickPointLocation.y << std::endl;//测试用
        }
    }

public:
    ROS() : Node("ros_node") {
        imageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
                "/raw_image", 10, std::bind( &ROS::image_callback, this, _1));
        clickPointPublisher = this->create_publisher<geometry_msgs::msg::Point32>( "/click_position", 10);
    }
};

cv::Point clickPointLocation;

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROS>());
    rclcpp::shutdown();
    return 0;
}
