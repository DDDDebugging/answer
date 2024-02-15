//
// Created by elsa on 24-2-14.
//

#include <answer/answer_node.h>

void Answer::clickPointLoc_callback(){
    auto message = geometry_msgs::msg::Point32();
        message.x = matchLocation.x;
        message.y = matchLocation.y;
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

    if(mode == 0){  //EZ难度
        if(abs( matchLocation.y - (lines[0][1] + lines[1][1]) / 2) <= 128){//发送模拟点击坐标
            clickPointLoc_callback();
        }
    }
    else{
        if(turned_matchLocation.y - (turned_lines[0][1] + turned_lines[1][1]) <= 115.0 * pro){
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

//获取旋转后判定线坐标
void Answer::turned_line_detector(cv::Mat & src_dst){
    cv::Mat dst;
    cv::Canny( src_dst, dst, 100, 200);
    cv::HoughLinesP( dst, turned_lines, 1, CV_PI/180, 10, 500, 22);

    /*for( cv::Vec< int, 4> n : turned_lines)
        std::cout << n;
    std::cout << "\n"; //测试用*/
}

//识别音符坐标（模板匹配）
void Answer::note_detector(){
    cv::Mat src,src_dst; //存放临时旋转后的模板
    image.copyTo(src);
    int w = src.cols;
    int h = src.rows;
    int nw,nh;
    cv::Mat M;
    //原图旋转
    if(lines[0][1] < lines[0][3]){
        angle = std::atan((lines[0][3] - lines[0][1]) * 1.0 / (lines[0][2] - lines[0][0]));
        M = cv::getRotationMatrix2D( cv::Point2f( src.cols * 1.0 / 2, src.rows * 1.0 / 2), angle / CV_PI * 180, 1.0);
        double cos = std::abs(M.at<double>(0,0));
        double sin = std::abs(M.at<double>(0,1));
        nw = src.cols * cos + src.rows * sin;
        nh = src.cols * sin + src.rows * cos;
        M.at<double>(0,2) += (nw / 2 - src.cols / 2);
        M.at<double>(1,2) += (nh / 2 - src.rows / 2);
        cv::warpAffine( src, src_dst, M, cv::Size( nw, nh));
        cv::resize( tmp, tmp, cv::Size(150.0 / nw * w, 18.0 / nh * h));
        mode = 1; //判定线左高右低
        turned_line_detector(src_dst);//获取旋转后判定线的位置
        pro = h * 1.0 / nh;
    }
    else if(lines[0][1] > lines[0][3]) {
        angle = std::atan((lines[0][1] - lines[0][3]) * 1.0 / (lines[0][2] - lines[0][0]));
        M = cv::getRotationMatrix2D(cv::Point2f(src.cols * 1.0 / 2, src.rows * 1.0 / 2), -angle / CV_PI * 180, 1.0);
        double cos = std::abs(M.at<double>(0, 0));
        double sin = std::abs(M.at<double>(0, 1));
        nw = src.cols * cos + src.rows * sin;
        nh = src.cols * sin + src.rows * cos;
        M.at<double>(0, 2) += (nw / 2 - src.cols / 2);
        M.at<double>(1, 2) += (nh / 2 - src.rows / 2);
        cv::warpAffine(src, src_dst, M, cv::Size(nw, nh));
        cv::resize(tmp, tmp, cv::Size(150.0 / nw * w, 18.0 / nh * h));
        mode = 2; //判定线左低右高
        turned_line_detector(src_dst);//获取旋转后判定线的位置
        pro = h * 1.0 / nh;
    }
    else{
        cv::resize( tmp, tmp, cv::Size( 150, 15));
        mode = 0;//EZ难度
    }

    //模板匹配
    cv::Mat dst;
    cv::Canny( image, dst, 100, 200, 3 );
    if(mode != 0){
        int result_rows = src_dst.rows - tmp.rows + 1;
        int result_cols = src_dst.cols - tmp.cols + 1;
        cv::Mat result( result_cols, result_rows, CV_32FC1);
        cv::matchTemplate( src_dst, tmp, result, cv::TM_CCOEFF_NORMED);
        cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        double minValue,maxValue;
        cv::Point minLocation,maxLocation;
        cv::minMaxLoc( result, &minValue, &maxValue, &minLocation, &maxLocation, cv::Mat());
        turned_matchLocation = maxLocation;

        std::cout << "turned_matchLocation: " << turned_matchLocation << std::endl; //测试用
        cv::circle( src_dst, cv::Point(turned_matchLocation.x + 80, turned_matchLocation.y), 2, cv::Scalar( 0, 0, 255), -1);
        cv::imwrite("test1.jpg",src_dst);

        //反推回在原图中的匹配坐标
        if(mode == 1){
            float dx = M.at<double>(0,2);
            float dy = M.at<double>(1,2);
            float x = (turned_matchLocation.x - dx) * cos(-angle * CV_PI / 180) - (turned_matchLocation.y - dy) * sin(-angle * CV_PI / 180) + dx;
            float y = (turned_matchLocation.x - dx) * sin(-angle * CV_PI / 180) + (turned_matchLocation.y - dy) * cos(-angle * CV_PI / 180) + dy;
            x = x / nw * w;
            y = y / nh * h;
            matchLocation.x = x + 80;
            matchLocation.y = y;
            //测试用
            //std::cout << "matchLocation: " << matchLocation << std::endl;
            /*cv::circle( src, cv::Point(x + 80,y), 2, cv::Scalar( 0, 0, 255), -1);
            cv::imwrite("test2.jpg",src);*/
        }
        else if(mode == 2){
            float dx = M.at<double>(0,2);
            float dy = M.at<double>(1,2);
            float x = (turned_matchLocation.x - dx) * cos(angle * CV_PI / 180) - (turned_matchLocation.y - dy) * sin(angle * CV_PI / 180) + dx;
            float y = (turned_matchLocation.x - dx) * sin(angle * CV_PI / 180) + (turned_matchLocation.y - dy) * cos(angle * CV_PI / 180) + dy;
            x = x / nw * w;
            y = y / nh * h;
            matchLocation.x = x;
            matchLocation.y = y + 40;
            //测试用
            //std::cout << "matchLocation: " << matchLocation << std::endl;
            /*cv::circle( src, cv::Point(x + 80,y), 2, cv::Scalar( 0, 0, 255), -1);
            cv::imwrite("test2.jpg",src);*/
        }
    }
    else{
        int result_rows = image.rows - tmp.rows + 1;
        int result_cols = image.cols - tmp.cols + 1;
        cv::Mat result( result_cols, result_rows, CV_32FC1);
        cv::matchTemplate( image, tmp, result, cv::TM_CCOEFF_NORMED);
        cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        double minValue,maxValue;
        cv::Point minLocation,maxLocation;
        cv::minMaxLoc( result, &minValue, &maxValue, &minLocation, &maxLocation, cv::Mat());
        matchLocation = maxLocation;
        matchLocation.x += 60;
        std::cout << "matchLocation: " << matchLocation << std::endl; //测试用
    }
}

Answer::Answer() : Node("answer_node") {
    imageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/raw_image", 10, std::bind( &Answer::image_callback, this, _1));
    clickPointPublisher = this->create_publisher<geometry_msgs::msg::Point32>( "/click_position", 10);
    mode = 0;
    angle = 0;
    matchLocation = cv::Point(0,0);
    turned_matchLocation = cv::Point(0,0);
    pro = 1.0;
}