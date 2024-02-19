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
        message.x = matchLocation.x + 20;
        message.y = matchLocation.y - 30;
    }
    else{
        message.x = matchLocation.x;
        message.y = matchLocation.y - 30;
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
        if(abs( matchLocation.y - (lines[0][1] + lines[1][1]) / 2) <= 100)//发送模拟点击坐标
            clickPointLoc_callback();
    }
    else{
        if(getDistance() <= 131)
            clickPointLoc_callback();
    }
}

//获取判定线坐标
void Answer::line_detector(){
    cv::Mat dst;
    cv::Canny( image, dst, 100, 200);
    cv::HoughLinesP( dst, lines, 1, CV_PI/180, 10, 200, 22);
    /*std::cout << "lines: " << std::endl;
    for( cv::Vec< int, 4> n : lines)
        std::cout << n;
    std::cout << "\n"; //测试用*/
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
        M = cv::getRotationMatrix2D(cv::Point2f(w * 1.0 / 2, h * 1.0 / 2), -angle / CV_PI * 180, 1.0);
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
        /*std::cout << "turned_matchLocation: " << turned_matchLocation << std::endl;
        cv::circle( src_dst, cv::Point(turned_matchLocation.x, turned_matchLocation.y), 2, cv::Scalar( 0, 0, 255), -1);
        cv::imwrite("test2.jpg",src_dst);*/
    }
    else{
        int result_rows = src.rows - tmp.rows + 1;
        int result_cols = src.cols - tmp.cols + 1;
        cv::Mat result( result_cols, result_rows, CV_32FC1);
        cv::matchTemplate( src, tmp, result, cv::TM_CCOEFF_NORMED);
        cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        int i,j;
        for( i = 0; i < result_cols; i++){
            for( j = 0; j < result_rows; j++){
                if(result.at<float>(j,i) > 0.85){
                    if(j > matchLocation.y){
                        matchLocation.y = j;
                        matchLocation.x = i;
                    }
                }
            }
        }
        /*std::cout << "matchLocation: " << matchLocation <<std::endl;
        cv::circle( src, cv::Point(matchLocation.x, matchLocation.y), 2, cv::Scalar( 0, 0, 255), -1);
        cv::imwrite("test2.jpg",src);*/
    }
    //std::cout << "angle: " << angle << std::endl;
    //反推回在原图中的匹配坐标
    if(mode == 1){
        float dx = nw / 2;
        float dy = nh / 2;
        float x,y;
        x = (turned_matchLocation.x - dx) * cos(-angle * CV_PI / 180) - (turned_matchLocation.y - dy) * sin(-angle * CV_PI / 180) + dx;
        y = (turned_matchLocation.x - dx) * sin(-angle * CV_PI / 180) + (turned_matchLocation.y - dy) * cos(-angle * CV_PI / 180) + dy;
        if(x >= nw / 2){
            x -= (nw - w) / 2;
            y -= (nh - h) / 2;
            matchLocation.x = x;
            matchLocation.y = y + angle * 823 * abs(matchLocation.x - w / 2) / (w / 2);
        }
        else{
            x -= (nw - w) / 2;
            y -= (nh - h) / 2;
            matchLocation.x = x;
            matchLocation.y = y - angle * 645 * abs(matchLocation.x - w / 2) / (w / 2);
        }
        //测试
        //std::cout << "matchLocation: " << matchLocation << std::endl;
        cv::circle( src, cv::Point(matchLocation.x, matchLocation.y), 3, cv::Scalar( 0, 0, 255), -1);
        cv::imwrite("test3.jpg",src);
    }
    else if(mode == 2){ //逆时针转回去
        float dx = nw / 2;
        float dy = nh / 2;
        float x = (turned_matchLocation.x - dx) * cos(angle * CV_PI / 180) - (turned_matchLocation.y - dy) * sin(angle * CV_PI / 180) + dx;
        float y = (turned_matchLocation.x - dx) * sin(angle * CV_PI / 180) + (turned_matchLocation.y - dy) * cos(angle * CV_PI / 180) + dy;
        if(x <= nw / 2){
            x -= (nw - w) / 2;
            y -= (nh - h) / 2;
            matchLocation.x = x;
            matchLocation.y = y + angle * 823 * abs(matchLocation.x - w / 2) / (w / 2);
        }
        else{
            x -= (nw - w) / 2;
            y -= (nh - h) / 2;
            matchLocation.x = x;
            matchLocation.y = y - angle * 645 * abs(matchLocation.x - w / 2) / (w / 2);
        }
        //测试用
        //std::cout << "matchLocation: " << matchLocation << std::endl;
        cv::circle( src, cv::Point(matchLocation.x, matchLocation.y), 3, cv::Scalar( 0, 0, 255), -1);
        cv::imwrite("test3.jpg",src);
    }
}

float Answer::getDistance(){
    float distance = 0;
    int A = 0, B = 0, C = 0;
    A = lines[0][1] - lines[0][3];
    B = lines[0][2] - lines[0][0];
    C = lines[0][0] * lines[0][3] - lines[0][1] * lines[0][2];
    distance = ((float)abs( A * matchLocation.x + B * matchLocation.y + C) / ((float) sqrtf(A * A + B * B)));
    std::cout << "distance: " << distance << std::endl;
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