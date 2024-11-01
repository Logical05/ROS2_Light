#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
    public:
        MyNode() : Node("cpp_node") {
            RCLCPP_INFO(this->get_logger(), "Test C++ Node");
        }

    private:
};