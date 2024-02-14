#include <answer/answer_node.h>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Answer>());
    rclcpp::shutdown();
    return 0;
}
