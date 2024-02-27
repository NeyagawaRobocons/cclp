#include <rclcpp/rclcpp.hpp>
#include <raylib.h>
#include <raymath.h>
#include <iostream>
#include <random>
#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cclp/msg/line_array.hpp"
#include "lines_and_points.hpp"

class Monitor : public rclcpp::Node
{
public:
    Monitor(): Node("monitor")
    {
        // Declare parameters
        std::string laser_scan = this->declare_parameter("laser_scan_topic", "scan");
        std::string line_map = this->declare_parameter("line_map_topic", "line_map");
        std::string map_frame = this->declare_parameter("map_frame", "map");
        std::string base_frame = this->declare_parameter("base_frame", "corrected_base_link");
        map_frame_ = map_frame;
        base_frame_ = base_frame;
        // Create a subscription to the laser scan
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_scan, 10, std::bind(&Monitor::laser_scan_callback, this, std::placeholders::_1));
        // Create a subscription to the line map
        line_map_sub_ = this->create_subscription<cclp::msg::LineArray>(
            line_map, 10, std::bind(&Monitor::line_map_callback, this, std::placeholders::_1));
        // Create a tf buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Start the UI thread
        ui_thread_ = std::thread(&Monitor::ui_main, this);

        RCLCPP_INFO(this->get_logger(), "Monitor node started");
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<cclp::msg::LineArray>::SharedPtr line_map_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_;
    std::string base_frame_;
    std::mutex points_mutex;
    std::vector<Vector2> points;
    std::mutex lines_mutex;
    std::vector<Line> lines;
    std::thread ui_thread_;

    void ui_main()
    {
        // Initialize the window
        int screenWidth = 1280;
        int screenHeight = 720;

        SetConfigFlags(FLAG_WINDOW_RESIZABLE);
        SetConfigFlags(FLAG_MSAA_4X_HINT);
        InitWindow(screenWidth, screenHeight, "cord correction monitor");
        screenWidth = GetScreenWidth();
        screenHeight = GetScreenHeight();

        SetTargetFPS(60);
        while (!WindowShouldClose())
        {
            screenWidth = GetScreenWidth();
            screenHeight = GetScreenHeight();

            geometry_msgs::msg::TransformStamped transform;
            try{
            transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            }catch(tf2::TransformException &ex){
                RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
                continue;
            }
            Quaternion q = {transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w};
            Vector3 tf_vec3 = {-transform.transform.translation.x, -transform.transform.translation.y, -QuaternionToEuler(q).z};

            std::vector<Vector2> moved_points;
            {
                std::lock_guard<std::mutex> lock(points_mutex);
                moved_points = points;
            }
            move_points(moved_points, tf2d_from_vec3(tf_vec3));
            // Clear the window
            BeginDrawing();
                ClearBackground(RAYWHITE);
                // Draw the points
                draw_points_scale(moved_points, 100, {(float)screenWidth / 2, (float)screenHeight / 2}, BLUE);
                // Draw the lines
                {
                    std::lock_guard<std::mutex> lock(lines_mutex);
                    draw_lines_scale(lines, 100, {(float)screenWidth / 2, (float)screenHeight / 2});
                }
                std::stringstream ss;
                ss << "FPS" << GetFPS() << std::endl << std::endl;
                ss << "Transform: " << tf_vec3.x << ", " << tf_vec3.y << ", " << tf_vec3.z << std::endl;
                ss << "Points: " << moved_points.size() << std::endl;
                ss << "Lines: " << lines.size() << std::endl;
                DrawText(ss.str().c_str(), 10, 10, 20, GRAY);
                // End drawing
            EndDrawing();
        }
        // Close the window
        CloseWindow();
        rclcpp::shutdown();
    }

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(points_mutex);
        points.clear();
        points.reserve(msg->ranges.size());
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
            {
                float angle = msg->angle_min + i * msg->angle_increment;
                Vector2 point = {msg->ranges[i] * cos(angle), msg->ranges[i] * sin(angle)};
                points.push_back(point);
            }
        }
    }

    void line_map_callback(const cclp::msg::LineArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(lines_mutex);
        lines.clear();
        lines.reserve(msg->lines.size());
        for (auto line : msg->lines)
        {
            Line l;
            l.from = {line.p1.x, line.p1.y};
            l.to = {line.p2.x, line.p2.y};
            lines.push_back(l);
        }
        RCLCPP_INFO(this->get_logger(), "Received %d lines", lines.size());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Monitor>());
    rclcpp::shutdown();
    return 0;
}
