//
// Created by elsa on 24-2-14.
//

#include <answer/answer_node.h>

void Answer::clickPointLoc_callback(){
    auto message = geometry_msgs::msg::Point32();
    if(mode != 2){
        message.x = matchLocation.x + 40;
        message.y = matchLocation.y;
    }
    else{
        message.x = matchLocation.x - 40;
        message.y = matchLocation.y;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Send position: (" << message.x << " " << message.y << ")");
    clickPointPublisher->publish(message);
}

void Answer::image_callback(const sensor_msgs::msg::Image &msg) {
    cv_bridge::CvImagePtr cvImage;
    cvImage = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8);
    cvImage->image.copyTo(image); //获取ROS传来的图片
    //image_threshold();//对图片和模板click进行二值化处理

    line_detector();//获取判定线坐标
    note_detector();//识别音符坐标（模板匹配）

    if(mode == 0){  //EZ难度
        if(abs( matchLocation.y - (lines[0][1] + lines[1][1]) / 2) <= 119){//发送模拟点击坐标
            clickPointLoc_callback();
        }
    }
    else if(mode == 1){  //判定线左高右低
        double point_height,line_height;
        point_height = matchLocation.y - matchLocation.x * tan(angle);
        line_height = lines[0][1] - lines[0][0] * tan(angle);
        if(abs(line_height - point_height) * cos(angle) <= 118){
            clickPointLoc_callback();
        }
    }
    else{  //判定线左低右高
        double point_height,line_height;
        point_height = matchLocation.y + matchLocation.x * tan(angle);
        line_height = lines[0][1] + lines[0][0] * tan(angle);
        if(abs(line_height - point_height) * cos(angle) <= 118){
            clickPointLoc_callback();
        }
    }
}

//获取判定线坐标
void Answer::line_detector(){
    cv::Mat dst;
    cv::Canny( image, dst, 100, 200);
    cv::HoughLinesP( dst, lines, 1, CV_PI/180, 10, 500, 22);

    /*for( cv::Vec< int, 4> n : lines)
        std::cout << n;
    std::cout << "\n"; //测试用*/
}

//识别音符坐标（模板匹配）
void Answer::note_detector(){
    cv::Mat tmp = cv::imread("click.png");
    //cv::cvtColor( tmp, tmp, cv::COLOR_BGR2GRAY);
    //cv::adaptiveThreshold( tmp, tmp, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 0);

    cv::Mat tmp_dst; //存放临时旋转后的模板
    if(lines[0][1] < lines[0][3]){
        angle = std::atan((lines[0][3] - lines[0][1]) * 1.0 / (lines[0][2] - lines[0][0]));
        cv::Mat M;
        M = cv::getRotationMatrix2D( cv::Point2f( tmp.cols / 2, tmp.rows / 2), -angle / CV_PI * 180, 1.0);
        double cos = std::abs(M.at<double>(0,0));
        double sin = std::abs(M.at<double>(0,1));
        int nw = tmp.cols * cos + tmp.rows * sin;
        int nh = tmp.cols * sin + tmp.rows * cos;
        M.at<double>(0,2) += (nw / 2 - tmp.cols / 2);
        M.at<double>(1,2) += (nh / 2 - tmp.rows / 2);
        cv::warpAffine( tmp, tmp_dst, M, cv::Size( nw, nh));
        cv::resize( tmp_dst, tmp_dst, cv::Size(130,18));
        mode = 1;
    }
    else if(lines[0][1] > lines[0][3]) {
        angle = std::atan((lines[0][1] - lines[0][3]) * 1.0 / (lines[0][2] - lines[0][0]));
        cv::Mat M;
        M = cv::getRotationMatrix2D(cv::Point2f(tmp.cols / 2, tmp.rows / 2), angle / CV_PI * 180, 1.0);
        double cos = std::abs(M.at<double>(0, 0));
        double sin = std::abs(M.at<double>(0, 1));
        int nw = tmp.cols * cos + tmp.rows * sin;
        int nh = tmp.cols * sin + tmp.rows * cos;
        M.at<double>(0, 2) += (nw / 2 - tmp.cols / 2);
        M.at<double>(1, 2) += (nh / 2 - tmp.rows / 2);
        cv::warpAffine(tmp, tmp_dst, M, cv::Size(nw, nh));
        cv::resize(tmp_dst, tmp_dst, cv::Size(130, 18));
        mode = 2;
    }
    else{
        cv::resize( tmp, tmp_dst, cv::Size( 150, 15));
        mode = 0;
    }
    //模板匹配
    cv::Mat dst;
    cv::Canny( image, dst, 100, 200, 3 );
    int result_rows = image.rows - tmp_dst.rows + 1;
    int result_cols = image.cols - tmp_dst.cols + 1;
    cv::Mat result( result_cols, result_rows, CV_32FC1);
    cv::matchTemplate( image, tmp_dst, result, cv::TM_CCOEFF_NORMED);
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    double minValue,maxValue;
    cv::Point minLocation,maxLocation;
    cv::minMaxLoc( result, &minValue, &maxValue, &minLocation, &maxLocation, cv::Mat());
    matchLocation = maxLocation;

    std::cout << "matchLocation: " << matchLocation << std::endl; //测试用
}

//对图片进行二值化处理
void Answer::image_threshold(){
    cv::cvtColor( image, image, cv::COLOR_BGR2GRAY);
    cv::adaptiveThreshold( image, image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 0);
}

Answer::Answer() : Node("answer_node") {
    imageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/raw_image", 10, std::bind( &Answer::image_callback, this, _1));
    clickPointPublisher = this->create_publisher<geometry_msgs::msg::Point32>( "/click_position", 10);
}