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

#include "lines_and_points.hpp"

class CordCorrectionNode : public rclcpp::Node
{
public:
    CordCorrectionNode(/* args */) : Node("cord_correction_node")
    {
        RCLCPP_INFO(this->get_logger(), "cord_correction_node is initializing...");
        // declare parameters
        this->declare_parameter<std::string>("map_frame_id", "map");
        this->declare_parameter<std::string>("parent_frame_id", "odom_frame");
        this->declare_parameter<std::string>("child_frame_id", "base_link");
        this->declare_parameter<std::string>("output_topic", "robot_pose");
        this->declare_parameter<std::string>("laser_scan_topic", "scan");
        this->declare_parameter<double>("publish_rate", 20.0);
        this->declare_parameter<int>("calc_per_loop", 20);
        this->declare_parameter<double>("gradient_delta", 0.02);
        parent_frame_id_ = this->get_parameter("parent_frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();
        gradient_delta_ = this->get_parameter("gradient_delta").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        calc_per_loop_ = this->get_parameter("calc_per_loop").as_int();
        auto output_topic = this->get_parameter("output_topic").as_string();
        auto laser_scan_topic = this->get_parameter("laser_scan_topic").as_string();

        // display parameters
        RCLCPP_INFO(this->get_logger(), "parent_frame_id: %s", parent_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "child_frame_id: %s", child_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "publish_rate: %f", publish_rate_);

        // tfのサブスクライバーの初期化
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic, 10);
        laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_scan_topic, 10, std::bind(&CordCorrectionNode::LaserScanCallback, this, std::placeholders::_1));
        initial_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "initial_pose", 10, std::bind(&CordCorrectionNode::InitialPoseCallback, this, std::placeholders::_1));
        
        StartTFThread();

        RCLCPP_INFO(this->get_logger(), "cord_correction_node has been initialized");
    }
private:
    std::string parent_frame_id_;
    std::string child_frame_id_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr initial_pose_subscription_;
    std::thread tf_thread_;
    std::mutex laser_data_mutex_;
    std::vector<Vector2> laser_data_;
    rclcpp::Time last_laser_data_time_;
    double publish_rate_;
    int calc_per_loop_;
    std::mutex tf_vec_mutex_;
    Vector3 tf_vec;

    float gradient_delta_;
    std::vector<Line> map_lines_;

    void StartTFThread(){
        tf_thread_ = std::thread([this](){
            rclcpp::Rate rate(publish_rate_);
            while(rclcpp::ok()){
                geometry_msgs::msg::TransformStamped transform;
                std::vector<Vector2> laser_data;
                rclcpp::Time last_laser_data_time;
                try
                {
                    std::lock_guard<std::mutex> lock(laser_data_mutex_);
                    laser_data = laser_data_;
                    last_laser_data_time = last_laser_data_time_;
                    transform = tf_buffer_->lookupTransform("map", parent_frame_id_, last_laser_data_time);
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
                laser_data = Vector2Transform(laser_data, tf2d_from_vec3({(float)transform.transform.translation.x, (float)transform.transform.translation.y, (float)transform.transform.rotation.z}));
                float delta = 0.02;
                Vector3 tf_vec_cp;
                {
                    std::lock_guard<std::mutex> lock(tf_vec_mutex_);
                    tf_vec_cp = tf_vec;
                }
                std::lock_guard<std::mutex> lock(tf_vec_mutex_);
                for (size_t i = 0; i < calc_per_loop_; i++)
                {
                    auto points = Vector2Transform(laser_data, tf2d_from_vec3(tf_vec_cp));
                    auto grad = grad_ave_distance_points_to_lines(points, map_lines_, delta);
                    tf_vec_cp = Vector3Multiply(grad, {-0.02, -0.02, -0.02});
                }
                {
                    std::lock_guard<std::mutex> lock(tf_vec_mutex_);
                    tf_vec = tf_vec_cp;
                }

                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = last_laser_data_time;
                tf_msg.header.frame_id = "map";
                tf_msg.child_frame_id = child_frame_id_;
                tf_msg.transform.translation.x = tf_vec_cp.x;
                tf_msg.transform.translation.y = tf_vec_cp.y;
                tf_msg.transform.translation.z = 0;
                auto rot = QuaternionFromEuler(tf_vec_cp.z, 0, 0);
                tf_msg.transform.rotation.x = rot.x;
                tf_msg.transform.rotation.y = rot.y;
                tf_msg.transform.rotation.z = rot.z;
                tf_msg.transform.rotation.w = rot.w;
                tf_broadcaster_->sendTransform(tf_msg);

                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = last_laser_data_time;
                pose_msg.header.frame_id = "map";
                pose_msg.pose.position.x = tf_vec_cp.x;
                pose_msg.pose.position.y = tf_vec_cp.y;
                pose_msg.pose.position.z = 0;
                pose_msg.pose.orientation.x = rot.x;
                pose_msg.pose.orientation.y = rot.y;
                pose_msg.pose.orientation.z = rot.z;
                pose_msg.pose.orientation.w = rot.w;
                pose_publisher_->publish(pose_msg);

                rate.sleep();
            }
        });

    }

    void LaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        std::lock_guard<std::mutex> lock(laser_data_mutex_);
        laser_data_.clear();
        laser_data_.reserve(msg->ranges.size());
        for(unsigned int i = 0; i < msg->ranges.size(); i++){
            if(std::isinf(msg->ranges[i])) continue;
            float angle = msg->angle_min + msg->angle_increment * i;
            float x = msg->ranges[i] * std::cos(angle);
            float y = msg->ranges[i] * std::sin(angle);
            laser_data_.push_back({x, y});
        }
        last_laser_data_time_ = msg->header.stamp;
    }

    void InitialPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        try
        {
            auto transform = tf_buffer_->lookupTransform("map", parent_frame_id_, tf2::TimePointZero);
            std::lock_guard<std::mutex> lock(tf_vec_mutex_);
            auto msg_rot = QuaternionToEuler({msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w});
            auto tf_rot = QuaternionToEuler({transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w});
            tf_vec = {msg->position.x - transform.transform.translation.x,
                msg->position.y - transform.transform.translation.y,
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
