#pragma once

#include "fmt_logger.hpp"
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class GripperNode : public rclcpp::Node
{
    using GripperCommand    = control_msgs::action::GripperCommand;
    using GoalHandleCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

public:
    GripperNode(const std::string &node_name);

    void SendGripperCmd(double normalized_width, double max_effort);

    double GetCloseJointPos() const
    {
        return close_joint_pos_;
    }

    double GetOpenJointPos() const
    {
        return open_joint_pos_;
    }

    double JointPosToNormalizedWidth(double joint_pos) const
    {
        return (joint_pos - close_joint_pos_) / (open_joint_pos_ - close_joint_pos_);
    }

    double NormalizedWidthToJointPos(double normalized_width) const
    {
        return (open_joint_pos_ - close_joint_pos_) * normalized_width + close_joint_pos_;
    }

private:
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
    FmtLogger logger_{this->get_logger()};
    double close_joint_pos_;
    double open_joint_pos_;

    void goal_response_callback(const GoalHandleCommand::SharedPtr &goal_handle);

    void feedback_callback(GoalHandleCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback);

    void result_callback(const GoalHandleCommand::WrappedResult &result);
};
