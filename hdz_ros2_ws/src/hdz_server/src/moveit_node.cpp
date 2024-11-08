#include "moveit_node.hpp"
#include <fmt/core.h>
#include <fmt/ranges.h>

MoveItNode::MoveItNode(const std::string &node_name)
    : Node(node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    // this->declare_parameter("planning_group", "ur_manipulator");
    // this->declare_parameter("max_velocity_scaling_factor", 0.05);
    // this->declare_parameter("max_acceleration_scaling_factor", 0.05);

    planning_group_                  = this->get_parameter("planning_group").as_string();
    max_velocity_scaling_factor_     = this->get_parameter("max_velocity_scaling_factor").as_double();
    max_acceleration_scaling_factor_ = this->get_parameter("max_acceleration_scaling_factor").as_double();
    logger_.Info("Planning group: {}", planning_group_);
}

void MoveItNode::Init()
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);

    logger_.Info("Max velocity scaling factor: {}", max_velocity_scaling_factor_);
    logger_.Info("Max acceleration scaling factor: {}", max_acceleration_scaling_factor_);
    logger_.Info("Planning frame: {}", move_group_->getPlanningFrame().c_str());
    logger_.Info("End effector link: {}", move_group_->getEndEffectorLink().c_str());
    logger_.Info("Available Planning Groups: {}", move_group_->getJointModelGroupNames());
}

void MoveItNode::SetTargetByName(const std::string &target)
{
    logger_.Info("Setting target to {}", target);
    move_group_->setNamedTarget(target);
}

void MoveItNode::PlanAndMove()
{
    logger_.Info("Planning the motion");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto planing_result = move_group_->plan(my_plan);

    if (planing_result != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("Failed to plan the motion");
    }

    logger_.Info("Moving the robot");
    auto move_result = move_group_->execute(my_plan);

    if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("Failed to move the robot");
    }
}

void MoveItNode::SetTargetPose(const geometry_msgs::msg::Pose &target_pose)
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
