#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <fmt/core.h>
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

public:
    MoveItNode(const std::string &node_name)
        : Node(node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // this->declare_parameter("planning_group", "ur_manipulator");
        // this->declare_parameter("max_velocity_scaling_factor", 0.05);
        // this->declare_parameter("max_acceleration_scaling_factor", 0.05);
        planning_group_                  = this->get_parameter("planning_group").as_string();
        max_velocity_scaling_factor_     = this->get_parameter("max_velocity_scaling_factor").as_double();
        max_acceleration_scaling_factor_ = this->get_parameter("max_acceleration_scaling_factor").as_double();
        LogInfo("Planning group: {}", planning_group_);
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void Init()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
        move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
        move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
        LogInfo("Max velocity scaling factor: {}", max_velocity_scaling_factor_);
        LogInfo("Max acceleration scaling factor: {}", max_acceleration_scaling_factor_);
    }

    void SetTargetByName(const std::string &target)
    {
        LogInfo("Setting target to {}", target);
        move_group_->setNamedTarget(target);
    }

    void PlanAndMove()
    {
        LogInfo("Planning the motion");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        auto planing_result = move_group_->plan(my_plan);

        if (planing_result != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Failed to plan the motion");
        }

        LogInfo("Moving the robot");
        auto move_result = move_group_->execute(my_plan);

        if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Failed to move the robot");
        }
    }

    auto GetCurrentPose(const std::string &end_effector_link = "") const
    {
        return move_group_->getCurrentPose(end_effector_link);
    }

    void SetTargetPose(const geometry_msgs::msg::Pose &target_pose)
    {
        moveit::core::RobotState target_robot_state(*(move_group_->getCurrentState()));
        const moveit::core::JointModelGroup *joint_model_group = move_group_->getCurrentState()->getJointModelGroup(planning_group_);

        auto ik_result = target_robot_state.setFromIK(joint_model_group, target_pose);
        // RCLCPP_INFO_STREAM(get_logger(), "ik_result: " << ik_result);
        if (!ik_result) {
            throw std::runtime_error("Failed to calculate IK solution");
        }

        std::vector<double> joint_group_positions;
        target_robot_state.copyJointGroupPositions(joint_model_group, joint_group_positions);

        // RCLCPP_INFO_STREAM(get_logger(), "Setting target pose to: ");
        // for (size_t i = 0; i < joint_group_positions.size(); ++i) {
        //     RCLCPP_INFO_STREAM(get_logger(), "Joint " << i << ": " << joint_group_positions[i]);
        // }

        move_group_->setJointValueTarget(joint_group_positions);
    }

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

    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::PoseStamped &msg, const std::string &prefix = "")
    {
        RCLCPP_INFO(logger, "%sheader: {frame_id: %s, stamp: %d.%d}\npose: {position: [%lf, %lf, %lf], orientation: [%lf, %lf, %lf, %lf]}",
                    prefix.c_str(), msg.header.frame_id.c_str(), msg.header.stamp.sec, msg.header.stamp.nanosec,
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    }

    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::TransformStamped &msg, const std::string &prefix = "")
    {
        RCLCPP_INFO(logger, "%sheader: {frame_id: %s, stamp: %d.%d}\ntransform: {translation: [%lf, %lf, %lf], rotation: [%lf, %lf, %lf, %lf]}",
                    prefix.c_str(), msg.header.frame_id.c_str(), msg.header.stamp.sec, msg.header.stamp.nanosec,
                    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z,
                    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w);
    }

    static void Print(const rclcpp::Logger &logger, const geometry_msgs::msg::Pose &msg, const std::string &prefix = "")
    {
        RCLCPP_INFO(logger, "%sposition: [%lf, %lf, %lf], orientation: [%lf, %lf, %lf, %lf]",
                    prefix.c_str(),
                    msg.position.x, msg.position.y, msg.position.z,
                    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    }

private:
    std::string planning_group_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    template <typename... Args>
    void LogInfo(const std::string &format_str, Args &&...args) const
    {
        auto formatted_str = fmt::format(format_str, std::forward<Args>(args)...);
        RCLCPP_INFO_STREAM(this->get_logger(), formatted_str);
    }
};
