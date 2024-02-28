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
#include <tf2_ros/transform_broadcaster.h>

#include "cclp/msg/line_array.hpp"
#include "lines_and_points.hpp"

class CordCorrectionNode : public rclcpp::Node
{
public:
    CordCorrectionNode(/* args */) : Node("cord_correction_node")
    {
        RCLCPP_INFO(this->get_logger(), "cord_correction_node is initializing...");
        // declare parameters
        this->declare_parameter<std::string>("map_frame_id", "map");
        this->declare_parameter<std::string>("baselink_frame_id", "base_link");
        this->declare_parameter<std::string>("corrected_frame_id", "corrected_base_link");
        this->declare_parameter<std::string>("output_topic", "robot_pose");
        this->declare_parameter<std::string>("map_topic", "line_map");
        this->declare_parameter<std::string>("laser1_scan_topic", "scan1");
        this->declare_parameter<std::string>("laser2_scan_topic", "scan2");
        this->declare_parameter<std::string>("initial_pose_topic", "initial_pose");
        this->declare_parameter<double>("publish_rate", 20.0);
        this->declare_parameter<int>("calc_per_loop", 15);
        this->declare_parameter<double>("gradient_delta", 0.02);
        // lidar1 parameters
        this->declare_parameter<double>("lidar1_offset_x", 0.2699);
        this->declare_parameter<double>("lidar1_offset_y", 0.2699);
        this->declare_parameter<double>("lidar1_offset_theta", -2.3561944901923);
        this->declare_parameter<bool>("lidar1_invert_x", true);
        this->declare_parameter<double>("lidar1_circle_mask_radius", 0.3);
        this->declare_parameter<double>("lidar1_circle_mask_center_x", 0.0);
        this->declare_parameter<double>("lidar1_circle_mask_center_y", 0.0);
        // lidar2 parameters
        this->declare_parameter<double>("lidar2_offset_x", -0.2699);
        this->declare_parameter<double>("lidar2_offset_y", -0.2699);
        this->declare_parameter<double>("lidar2_offset_theta", 0.78539816339745);
        this->declare_parameter<bool>("lidar2_invert_x", true);
        this->declare_parameter<double>("lidar2_circle_mask_radius", 0.3);
        this->declare_parameter<double>("lidar2_circle_mask_center_x", 0.0);
        this->declare_parameter<double>("lidar2_circle_mask_center_y", 0.0);
        map_frame_id_ = this->get_parameter("map_frame_id").as_string();
        baselink_frame_id_ = this->get_parameter("baselink_frame_id").as_string();
        corrected_frame_id_ = this->get_parameter("corrected_frame_id").as_string();
        gradient_delta_ = this->get_parameter("gradient_delta").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        calc_per_loop_ = this->get_parameter("calc_per_loop").as_int();
        // lidar1 parameters
        lidar1_offset_x_ = this->get_parameter("lidar1_offset_x").as_double();
        lidar1_offset_y_ = this->get_parameter("lidar1_offset_y").as_double();
        lidar1_offset_theta_ = this->get_parameter("lidar1_offset_theta").as_double();
        lidar1_invert_x_ = this->get_parameter("lidar1_invert_x").as_bool();
        lidar1_circle_mask_radius_ = this->get_parameter("lidar1_circle_mask_radius").as_double();
        lidar1_circle_mask_center_x_ = this->get_parameter("lidar1_circle_mask_center_x").as_double();
        lidar1_circle_mask_center_y_ = this->get_parameter("lidar1_circle_mask_center_y").as_double();
        // lidar2 parameters
        lidar2_offset_x_ = this->get_parameter("lidar2_offset_x").as_double();
        lidar2_offset_y_ = this->get_parameter("lidar2_offset_y").as_double();
        lidar2_offset_theta_ = this->get_parameter("lidar2_offset_theta").as_double();
        lidar2_invert_x_ = this->get_parameter("lidar2_invert_x").as_bool();
        lidar2_circle_mask_radius_ = this->get_parameter("lidar2_circle_mask_radius").as_double();
        lidar2_circle_mask_center_x_ = this->get_parameter("lidar2_circle_mask_center_x").as_double();
        lidar2_circle_mask_center_y_ = this->get_parameter("lidar2_circle_mask_center_y").as_double();
        auto output_topic = this->get_parameter("output_topic").as_string();
        auto map_topic = this->get_parameter("map_topic").as_string();
        auto laser1_scan_topic = this->get_parameter("laser1_scan_topic").as_string();
        auto laser2_scan_topic = this->get_parameter("laser2_scan_topic").as_string();
        auto initial_pose_topic = this->get_parameter("initial_pose_topic").as_string();

        // display parameters
        RCLCPP_INFO(this->get_logger(), "parent_frame_id: %s", map_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "baselink_frame_id: %s", baselink_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "publish_rate: %f", publish_rate_);

        // tfのサブスクライバーの初期化
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic, 10);
        map_subscription_ = this->create_subscription<cclp::msg::LineArray>(
            map_topic, 10, std::bind(&CordCorrectionNode::MapCallback, this, std::placeholders::_1));
        laser1_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser1_scan_topic, 10, std::bind(&CordCorrectionNode::Laser1ScanCallback, this, std::placeholders::_1));
        laser2_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser2_scan_topic, 10, std::bind(&CordCorrectionNode::Laser2ScanCallback, this, std::placeholders::_1));
        initial_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            initial_pose_topic, 10, std::bind(&CordCorrectionNode::InitialPoseCallback, this, std::placeholders::_1));
        
        point_tf_vec = {0, 0, 0};
        StartTFThread();

        RCLCPP_INFO(this->get_logger(), "cord_correction_node has been initialized");
    }
private:
    std::string map_frame_id_;
    std::string baselink_frame_id_;
    std::string corrected_frame_id_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<cclp::msg::LineArray>::SharedPtr map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser1_scan_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser2_scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr initial_pose_subscription_;
    std::thread tf_thread_;
    // laser1 data
    std::mutex laser1_data_mutex_;
    std::vector<Vector2> laser1_data_;
    rclcpp::Time last_laser1_data_time_;
    // laser2 data
    std::mutex laser2_data_mutex_;
    std::vector<Vector2> laser2_data_;
    rclcpp::Time last_laser2_data_time_;
    // parameters
    double publish_rate_;
    int calc_per_loop_;
    // lidar1 parameters
    double lidar1_offset_x_;
    double lidar1_offset_y_;
    double lidar1_offset_theta_;
    bool lidar1_invert_x_;
    double lidar1_circle_mask_radius_;
    double lidar1_circle_mask_center_x_;
    double lidar1_circle_mask_center_y_;
    // lidar2 parameters
    double lidar2_offset_x_;
    double lidar2_offset_y_;
    double lidar2_offset_theta_;
    bool lidar2_invert_x_;
    double lidar2_circle_mask_radius_;
    double lidar2_circle_mask_center_x_;
    double lidar2_circle_mask_center_y_;
    std::mutex point_tf_vec_mutex_;
    Vector3 point_tf_vec;

    float gradient_delta_;
    std::vector<Line> map_lines_;

    void StartTFThread(){
        tf_thread_ = std::thread([this](){
            rclcpp::Rate rate(publish_rate_);
            while(rclcpp::ok()){
                geometry_msgs::msg::TransformStamped transform;
                std::vector<Vector2> laser_data;
                rclcpp::Time last_laser_data_time;
                {std::lock_guard<std::mutex> lock(laser1_data_mutex_);
                laser_data = laser1_data_;
                last_laser_data_time = last_laser1_data_time_;}
                {std::lock_guard<std::mutex> lock2(laser2_data_mutex_);
                laser_data.insert(laser_data.end(), laser2_data_.begin(), laser2_data_.end());}
                try
                {
                    transform = tf_buffer_->lookupTransform(map_frame_id_, baselink_frame_id_, last_laser_data_time);
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
                RCLCPP_INFO(this->get_logger(), "points: %d", laser_data.size());
                float transform_rot = QuaternionToEuler({transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w}).z;
                laser_data = Vector2Transform(laser_data, tf2d_from_vec3({(float)transform.transform.translation.x, (float)transform.transform.translation.y, (float)transform.transform.rotation.z}));
                float delta = 0.02;
                Vector3 point_tf_vec_cp;
                {
                    std::lock_guard<std::mutex> lock(point_tf_vec_mutex_);
                    point_tf_vec_cp = point_tf_vec;
                    if(map_lines_.size() != 0) for (size_t i = 0; i < calc_per_loop_; i++)
                    {
                        auto points = Vector2Transform(laser_data, tf2d_from_vec3(point_tf_vec_cp));
                        auto grad = grad_ave_distance_points_to_lines(points, map_lines_, delta);
                        point_tf_vec_cp = Vector3Add(point_tf_vec_cp,  Vector3Multiply(grad, {-0.02, -0.02, -0.02}));
                    }
                    else
                    {
                        point_tf_vec_cp = {0, 0, 0};
                        RCLCPP_INFO(this->get_logger(), "line_map is empty");
                    }
                    if(std::isnan(point_tf_vec_cp.x) || std::isnan(point_tf_vec_cp.y) || std::isnan(point_tf_vec_cp.z)) point_tf_vec_cp = {0, 0, 0};
                    {
                        point_tf_vec = point_tf_vec_cp;
                    }
                }

                auto vec = Vector2Transform({point_tf_vec_cp.x, point_tf_vec_cp.y}, MatrixRotateZ(-transform_rot));
                auto rot = QuaternionFromEuler(0, 0, point_tf_vec_cp.z);
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = now();
                tf_msg.header.frame_id = baselink_frame_id_;
                tf_msg.child_frame_id = corrected_frame_id_;
                tf_msg.transform.translation.x = vec.x;
                tf_msg.transform.translation.y = vec.y;
                tf_msg.transform.translation.z = 0;
                tf_msg.transform.rotation.x = rot.x;
                tf_msg.transform.rotation.y = rot.y;
                tf_msg.transform.rotation.z = rot.z;
                tf_msg.transform.rotation.w = rot.w;
                tf_broadcaster_->sendTransform(tf_msg);

                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = now();
                pose_msg.header.frame_id = baselink_frame_id_;
                pose_msg.pose.position.x = vec.x;
                pose_msg.pose.position.y = vec.y;
                pose_msg.pose.position.z = 0;
                pose_msg.pose.orientation.x = rot.x;
                pose_msg.pose.orientation.y = rot.y;
                pose_msg.pose.orientation.z = rot.z;
                pose_msg.pose.orientation.w = rot.w;
                pose_publisher_->publish(pose_msg);

                rate.sleep();

                RCLCPP_INFO(this->get_logger(), "point_tf_vec: %f, %f, %f", point_tf_vec_cp.x, point_tf_vec_cp.y, point_tf_vec_cp.z);
            }
        });

    }

    void MapCallback(const cclp::msg::LineArray::SharedPtr msg){
        map_lines_.clear();
        map_lines_.reserve(msg->lines.size());
        for(auto line : msg->lines){
            map_lines_.push_back({{line.p1.x, line.p1.y}, {line.p2.x, line.p2.y}});
        }
    }


    void Laser1ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        std::lock_guard<std::mutex> lock(laser1_data_mutex_);
        laser1_data_.clear();
        laser1_data_.reserve(msg->ranges.size());
        for(unsigned int i = 0; i < msg->ranges.size(); i++){
            if(std::isinf(msg->ranges[i])) continue;
            if(std::isnan(msg->ranges[i])) continue;
            float angle = msg->angle_min + msg->angle_increment * i + lidar1_offset_theta_;
            float x = msg->ranges[i] * std::cos(angle) + lidar1_offset_x_;
            float y = msg->ranges[i] * std::sin(angle) + lidar1_offset_y_;
            if(lidar1_invert_x_) x = -x;
            if(std::pow(x - lidar1_circle_mask_center_x_, 2) + std::pow(y - lidar1_circle_mask_center_y_, 2) < std::pow(lidar1_circle_mask_radius_, 2)) continue;
            laser1_data_.push_back({x, y});
        }
        last_laser1_data_time_ = msg->header.stamp;
    }

    void Laser2ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        std::lock_guard<std::mutex> lock(laser2_data_mutex_);
        laser2_data_.clear();
        laser2_data_.reserve(msg->ranges.size());
        for(unsigned int i = 0; i < msg->ranges.size(); i++){
            if(std::isinf(msg->ranges[i])) continue;
            if(std::isnan(msg->ranges[i])) continue;
            float angle = msg->angle_min + msg->angle_increment * i + lidar2_offset_theta_;
            float x = msg->ranges[i] * std::cos(angle) + lidar2_offset_x_;
            float y = msg->ranges[i] * std::sin(angle) + lidar2_offset_y_;
            if(lidar2_invert_x_) x = -x;
            if(std::pow(x - lidar2_circle_mask_center_x_, 2) + std::pow(y - lidar2_circle_mask_center_y_, 2) < std::pow(lidar2_circle_mask_radius_, 2)) continue;
            laser2_data_.push_back({x, y});
        }
        last_laser2_data_time_ = msg->header.stamp;
    }

    void InitialPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        try
        {
            auto transform = tf_buffer_->lookupTransform(map_frame_id_, baselink_frame_id_, tf2::TimePointZero);
            std::lock_guard<std::mutex> lock(point_tf_vec_mutex_);
            auto msg_rot = QuaternionToEuler({msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w});
            auto tf_rot = QuaternionToEuler({transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w});
            point_tf_vec = {(float)msg->position.x - (float)transform.transform.translation.x,
                (float)msg->position.y - (float)transform.transform.translation.y,
                msg_rot.z - tf_rot.z};
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CordCorrectionNode>());
  rclcpp::shutdown();
  return 0;
}
