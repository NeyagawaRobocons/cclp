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
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

#include "cclp/msg/line_array.hpp"
#include "lines_and_points.hpp"

class Monitor : public rclcpp::Node
{
public:
    Monitor(): Node("monitor")
    {
        // Declare parameters
        std::string laser1_scan = this->declare_parameter("laser1_scan_topic", "scan1");
        std::string laser2_scan = this->declare_parameter("laser2_scan_topic", "scan2");
        std::string line_map = this->declare_parameter("line_map_topic", "line_map");
        std::string mouse_point_pub = this->declare_parameter("mouse_point_pub_topic", "initial_pose");
        map_frame_ = this->declare_parameter("map_frame", "map");
        base_frame_ = this->declare_parameter("base_frame", "corrected_base_link");

        // create a client to get parameters
        // get_parameters_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/cord_correction_node/get_parameters");
        // Create topic subscriptions and publishers
        laser1_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser1_scan, 10, std::bind(&Monitor::Laser1ScanCallback, this, std::placeholders::_1));
        laser2_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser2_scan, 10, std::bind(&Monitor::Laser2ScanCallback, this, std::placeholders::_1));
        line_map_sub_ = this->create_subscription<cclp::msg::LineArray>(
            line_map, 10, std::bind(&Monitor::line_map_callback, this, std::placeholders::_1));
        mouse_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(mouse_point_pub, 10);
        // Create a tf buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // get_params_wall_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(1000), std::bind(&Monitor::get_lidar_params, this));

        // Start the UI thread
        ui_thread_ = std::thread(&Monitor::ui_main, this);

        RCLCPP_INFO(this->get_logger(), "Monitor node started");
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser1_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser2_scan_sub_;
    rclcpp::Subscription<cclp::msg::LineArray>::SharedPtr line_map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr mouse_point_pub_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_;
    std::string base_frame_;
    std::mutex points1_mutex;
    std::vector<Vector2> points1;
    std::mutex points2_mutex;
    std::vector<Vector2> points2;
    std::mutex lines_mutex;
    std::vector<Line> lines;
    std::thread ui_thread_;
    rclcpp::TimerBase::SharedPtr get_params_wall_timer_;

    // lidar1 parameters
    double lidar1_offset_x_ = - 0.2699;
    double lidar1_offset_y_ = 0.2699;
    double lidar1_offset_theta_ = -0.78539816339745;
    bool lidar1_invert_x_ = true;
    double lidar1_circle_mask_radius_ = 0.5;
    double lidar1_circle_mask_center_x_ = 0.0;
    double lidar1_circle_mask_center_y_ = 0.0;
    // lidar2 parameters
    double lidar2_offset_x_ = 0.2699;
    double lidar2_offset_y_ = - 0.2699;
    double lidar2_offset_theta_ = 2.3561944901923;
    bool lidar2_invert_x_ = true;
    double lidar2_circle_mask_radius_ = 0.5;
    double lidar2_circle_mask_center_x_ = 0.0;
    double lidar2_circle_mask_center_y_ = 0.0;


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


        Vector2 mouse_map_begin = {0, 0};
        Vector2 mouse_map_end = {0, 0};

        SetTargetFPS(60);
        while (!WindowShouldClose() && rclcpp::ok())
        {
            screenWidth = GetScreenWidth();
            screenHeight = GetScreenHeight();

            float map_draw_scale = 100;
            Vector2 map_draw_origin = {(float)screenWidth / 2, (float)screenHeight - 400};

            Vector2 mouse_map_begin;
            if(IsMouseButtonPressed(MOUSE_LEFT_BUTTON)){
                Vector2 mouse = GetMousePosition();
                Vector2 mouse_map = Vector2Subtract(mouse, map_draw_origin);
                mouse_map = Vector2Divide(mouse_map, {map_draw_scale, -map_draw_scale});
                mouse_map_begin = mouse_map;
            }
            if(IsMouseButtonDown(MOUSE_LEFT_BUTTON)){
                Vector2 mouse = GetMousePosition();
                Vector2 mouse_map = Vector2Subtract(mouse, map_draw_origin);
                mouse_map = Vector2Divide(mouse_map, {map_draw_scale, -map_draw_scale});
                mouse_map_end = mouse_map;
            }
            if(IsMouseButtonReleased(MOUSE_LEFT_BUTTON)){
                geometry_msgs::msg::Pose pose;
                pose.position.x = mouse_map_begin.x;
                pose.position.y = mouse_map_begin.y;
                pose.position.z = 0;
                Quaternion q = QuaternionFromEuler(0, 0, std::atan2(mouse_map_end.y - mouse_map_begin.y, mouse_map_end.x - mouse_map_begin.x));
                pose.orientation.x = q.x;
                pose.orientation.y = q.y;
                pose.orientation.z = q.z;
                pose.orientation.w = q.w;
                mouse_point_pub_->publish(pose);
            }

            Vector3 tf_vec3;
            std::string err_tf;
            try{
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
                Quaternion q = {transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w};
                tf_vec3 = {transform.transform.translation.x, transform.transform.translation.y, QuaternionToEuler(q).z};
            }catch(tf2::TransformException &ex){
                err_tf = ex.what();
            }


            Vector3 tf_vec3_base;
            std::string err_tf_base;
            try{
                geometry_msgs::msg::TransformStamped transform_base = tf_buffer_->lookupTransform(map_frame_, "base_link", tf2::TimePointZero);
                Quaternion q_base = {transform_base.transform.rotation.x, transform_base.transform.rotation.y, transform_base.transform.rotation.z, transform_base.transform.rotation.w};
                tf_vec3_base = {transform_base.transform.translation.x, transform_base.transform.translation.y, QuaternionToEuler(q_base).z};
            }catch(tf2::TransformException &ex){
                err_tf_base = ex.what();
            }

            std::vector<Vector2> moved_points1;
            {
                std::lock_guard<std::mutex> lock(points1_mutex);
                moved_points1 = points1;
            }
            move_points(moved_points1, tf2d_from_vec3(tf_vec3));
            std::vector<Vector2> moved_points2;
            {
                std::lock_guard<std::mutex> lock(points2_mutex);
                moved_points2 = points2;
            }
            move_points(moved_points2, tf2d_from_vec3(tf_vec3));
            // Clear the window
            BeginDrawing();
                ClearBackground(RAYWHITE);
                // Draw the points
                draw_points_scale_y_inv(moved_points1, map_draw_scale, map_draw_origin, BLUE);
                draw_points_scale_y_inv(moved_points2, map_draw_scale, map_draw_origin, BLUE);
                // Draw the lines
                {
                    std::lock_guard<std::mutex> lock(lines_mutex);
                    draw_lines_scale_y_inv(lines, map_draw_scale, map_draw_origin);
                }
                // Draw the mouse line
                if(IsMouseButtonDown(MOUSE_LEFT_BUTTON)){
                    draw_line_scale_y_inv({mouse_map_begin, mouse_map_end}, map_draw_scale, map_draw_origin, RED);
                }
                // Draw the transform
                draw_tf_scale_y_inv(tf_vec3, map_draw_scale, map_draw_origin);
                draw_tf_scale_y_inv(tf_vec3_base, map_draw_scale, map_draw_origin);

                std::stringstream ss;
                ss << "FPS" << GetFPS() << std::endl << std::endl;
                ss << "Transform: " << tf_vec3.x << ", " << tf_vec3.y << ", " << tf_vec3.z << std::endl;
                ss << "Points1: " << moved_points1.size() << std::endl;
                ss << "Points2: " << moved_points2.size() << std::endl;
                ss << "Lines: " << lines.size() << std::endl;
                ss << "Error transform: " << err_tf << std::endl;
                ss << "Error base transorm: " << err_tf_base << std::endl;
                DrawText(ss.str().c_str(), 10, 10, 20, GRAY);
                // End drawing
            EndDrawing();
        }
        // Close the window
        CloseWindow();
        rclcpp::shutdown();
    }


   void Laser1ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        std::lock_guard<std::mutex> lock(points1_mutex);
        points1.clear();
        points1.reserve(msg->ranges.size());
        for(unsigned int i = 0; i < msg->ranges.size(); i++){
            if(std::isinf(msg->ranges[i])) continue;
            if(std::isnan(msg->ranges[i])) continue;
            float angle = msg->angle_min + msg->angle_increment * i + lidar1_offset_theta_;
            float x = msg->ranges[i] * std::cos(angle) + lidar1_offset_x_;
            float y = msg->ranges[i] * std::sin(angle) + lidar1_offset_y_;
            if(lidar1_invert_x_) x = -x;
            if(std::pow(x - lidar1_circle_mask_center_x_, 2) + std::pow(y - lidar1_circle_mask_center_y_, 2) < std::pow(lidar1_circle_mask_radius_, 2)) continue;
            points1.push_back({x, y});
        }
    }

    void Laser2ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        std::lock_guard<std::mutex> lock(points2_mutex);
        points2.clear();
        points2.reserve(msg->ranges.size());
        for(unsigned int i = 0; i < msg->ranges.size(); i++){
            if(std::isinf(msg->ranges[i])) continue;
            if(std::isnan(msg->ranges[i])) continue;
            float angle = msg->angle_min + msg->angle_increment * i + lidar2_offset_theta_;
            float x = msg->ranges[i] * std::cos(angle) + lidar2_offset_x_;
            float y = msg->ranges[i] * std::sin(angle) + lidar2_offset_y_;
            if(lidar2_invert_x_) x = -x;
            if(std::pow(x - lidar2_circle_mask_center_x_, 2) + std::pow(y - lidar2_circle_mask_center_y_, 2) < std::pow(lidar2_circle_mask_radius_, 2)) continue;
            points2.push_back({x, y});
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
