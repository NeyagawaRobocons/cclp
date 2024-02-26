#include <rclcpp/rclcpp.hpp>
#include <iostream>


int main()
{
    std::cout << "Hello, World!" << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello, World!");
    return 0;
}
