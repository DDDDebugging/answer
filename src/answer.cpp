//
// Created by elsa on 24-2-14.
//

#include <answer/answer_node.h>

void Answer::clickPointLoc_callback(){
    auto message = geometry_msgs::msg::Point32();
    if(mode == 0){
        message.x = matchLocation.x + 30;
        message.y = (lines[0][1] + lines[1][1]) / 2 - 10;
    }
    else if(mode == 1){
        message.x = matchLocation.x + 10;
        message.y = matchLocation.y;
    }
    else{
        message.x = matchLocation.x + 15;
        message.y = matchLocation.y - 15;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Send position: (" << message.x << " " << message.y << ")");
    clickPointPublisher->publish(message);
}

void Answer::image_callback(const sensor_msgs::msg::Image &msg) {
    cv_bridge::CvImagePtr cvImage;
    cvImage = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8);
    cvImage->image.copyTo(image); //获取ROS传来的图片

    line_detector();//获取原图判定线坐标
    note_detector();//识别音符坐标（模板匹配）

    if(mode == 0){//EZ难度
        if(abs( matchLocation.y - (lines[0][1] + lines[1][1]) / 2) <= 105)//发送模拟点击坐标
            clickPointLoc_callback();
    }
    else{
        if(getDistance() <= 130 * (0.9 + angle))
            clickPointLoc_callback();
    }
}

//获取判定线坐标
void Answer::line_detector(){
    cv::Mat dst;
    cv::Canny( image, dst, 100, 200);
    cv::HoughLinesP( dst, lines, 1, CV_PI/180, 10, 200, 22);
}

//识别音符坐标（模板匹配）
void Answer::note_detector(){
    cv::Mat src,src_dst; //存放临时旋转后的模板
    image.copyTo(src);
    float w = src.cols;
    float h = src.rows;
    float nw,nh;
    cv::Mat M;
    matchLocation = cv::Point(0,0);
    turned_matchLocation = cv::Point(0,0);
    //原图旋转
    if(lines[0][1] < lines[0][3]){
        angle = std::atan((lines[0][3] - lines[0][1]) * 1.0 / (lines[0][2] - lines[0][0]));
        M = cv::getRotationMatrix2D( cv::Point2f( w / 2, h / 2), angle / CV_PI * 180, 1.0);
        double cos = std::abs(M.at<double>(0,0));
        double sin = std::abs(M.at<double>(0,1));
        nw = w * cos + h * sin;
        nh = w * sin + h * cos;
        M.at<double>(0,2) += (nw / 2 - w / 2);
        M.at<double>(1,2) += (nh / 2 - h / 2);
        cv::warpAffine( src, src_dst, M, cv::Size( nw, nh));
        cv::resize( tmp, tmp, cv::Size(155, 16));
        mode = 1; //判定线左高右低
    }
    else if(lines[0][1] > lines[0][3]){
        angle = std::atan((lines[0][1] - lines[0][3]) * 1.0 / (lines[0][2] - lines[0][0]));
        M = cv::getRotationMatrix2D(cv::Point2f(w / 2, h / 2), -angle / CV_PI * 180, 1.0);
        double cos = std::abs(M.at<double>(0, 0));
        double sin = std::abs(M.at<double>(0, 1));
        nw = w * cos + h * sin;
        nh = w * sin + h * cos;
        M.at<double>(0, 2) += (nw / 2 - w / 2);
        M.at<double>(1, 2) += (nh / 2 - h / 2);
        cv::warpAffine(src, src_dst, M, cv::Size(nw, nh));
        cv::resize(tmp, tmp, cv::Size(155, 16));
        mode = 2; //判定线左低右高
    }
    else{
        cv::resize( tmp, tmp, cv::Size( 155, 16));
        mode = 0;//EZ难度
    }
    //模板匹配
    if(mode != 0){
        int result_rows = src_dst.rows - tmp.rows + 1;
        int result_cols = src_dst.cols - tmp.cols + 1;
        cv::Mat result( result_cols, result_rows, CV_32FC1);
        cv::matchTemplate( src_dst, tmp, result, cv::TM_CCOEFF_NORMED);
        cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        int i,j;
        for( i = 0; i < result_cols; i++){
            for( j = 0; j < result_rows; j++){
                if(result.at<float>(j,i) > 0.85){
                    if(j > turned_matchLocation.y){
                        turned_matchLocation.y = j;
                        turned_matchLocation.x = i;
                    }
                }
            }
        }
        turned_matchLocation.x += 20;
    }
    else {
        int result_rows = src.rows - tmp.rows + 1;
        int result_cols = src.cols - tmp.cols + 1;
        cv::Mat result(result_cols, result_rows, CV_32FC1);
        cv::matchTemplate(src, tmp, result, cv::TM_CCOEFF_NORMED);
        cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        int i, j;
        for (i = 0; i < result_cols; i++) {
            for (j = 0; j < result_rows; j++) {
                if (result.at<float>(j, i) > 0.85) {
                    if (j > matchLocation.y) {
                        matchLocation.y = j;
                        matchLocation.x = i;
                    }
                }
            }
        }
    }
    //反推回原图坐标
    if(mode == 1){
        float x,y;
        x = turned_matchLocation.x * cos(angle) - turned_matchLocation.y * sin(angle) + w * sin(angle) * sin(angle);
        y = turned_matchLocation.x * sin(angle) + turned_matchLocation.y * cos(angle) - w * sin(angle) * cos(angle);
        matchLocation.x = x;
        matchLocation.y = y;
    }
    else if(mode == 2){
        float x,y;
        x = turned_matchLocation.x * cos(angle) + turned_matchLocation.y * sin(angle) - h * sin(angle) * cos(angle);
        y = - turned_matchLocation.x * sin(angle) + turned_matchLocation.y * cos(angle) + h * sin(angle) * sin(angle);
        matchLocation.x = x;
        matchLocation.y = y;
    }
}

float Answer::getDistance(){
    float distance = 0;
    int x1 = lines[0][0], x2 = lines[0][2], y1 = lines[0][1], y2 = lines[0][3]; //取直线上两点
    distance = abs((y2 - y1) * (matchLocation.x - x1) + (y1 - matchLocation.y) * (x2 - x1)) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    return distance;
}

Answer::Answer() : Node("answer_node") {
    imageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/raw_image", 10, std::bind( &Answer::image_callback, this, _1));
    clickPointPublisher = this->create_publisher<geometry_msgs::msg::Point32>( "/click_position", 10);
    mode = 0;
    angle = 0;
    matchLocation.x =0;
    matchLocation.y =0;
    turned_matchLocation = cv::Point(0,0);
}
