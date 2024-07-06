#pragma once

#include "fmt_logger.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MoveItNode : public rclcpp::Node
{
public:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    double max_velocity_scaling_factor_     = 0.05;
    double max_acceleration_scaling_factor_ = 0.05;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    FmtLogger logger_{this->get_logger()};

public:
    MoveItNode(const std::string &node_name);

    void Init();

    void SetTargetByName(const std::string &target);

    void PlanAndMove();

    auto GetCurrentPose(const std::string &end_effector_link = "") const;

    void SetTargetPose(const geometry_msgs::msg::Pose &target_pose);

    auto GetJointPos() const
    {
        auto robot_state                                       = move_group_->getCurrentState();
        const moveit::core::JointModelGroup *joint_model_group = move_group_->getCurrentState()->getJointModelGroup(planning_group_);
        std::vector<double> joint_pos;
        robot_state->copyJointGroupPositions(joint_model_group, joint_pos);
        return joint_pos;
    }

    std::string GetPlanningGroup() const
    {
        return planning_group_;
    }

    template <typename T>
    void Print(const T &msg, const std::string &prefix = "")
    {
        Print(this->get_logger(), msg, prefix);
    }

    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::PoseStamped &msg, const std::string &prefix = "");
    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::TransformStamped &msg, const std::string &prefix = "");
    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::Pose &msg, const std::string &prefix = "");

private:
    std::string planning_group_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};
