#pragma once

#include "fmt_logger.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MoveItNode : public rclcpp::Node
{
public:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    double max_velocity_scaling_factor_     = 0.05;
    double max_acceleration_scaling_factor_ = 0.05;
    FmtLogger logger_{this->get_logger()};

public:
    MoveItNode(const std::string &node_name);

    void Init();

    void SetTargetByName(const std::string &target);

    void PlanAndMove();

    auto GetCurrentPose(const std::string &end_effector_link = "") const
    {
        return move_group_->getCurrentPose(end_effector_link);
    }

    void SetTargetPose(const geometry_msgs::msg::Pose &target_pose);

    auto GetJointPos() const
    {
        auto robot_state                                       = move_group_->getCurrentState();
        const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(planning_group_);
        std::vector<double> joint_pos;
        robot_state->copyJointGroupPositions(joint_model_group, joint_pos);
        return joint_pos;
    }

    std::string GetPlanningGroup() const
    {
        return planning_group_;
    }

private:
    std::string planning_group_;
};
