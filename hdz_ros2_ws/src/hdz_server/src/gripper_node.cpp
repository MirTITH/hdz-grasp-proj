#include "gripper_node.hpp"
#include <control_msgs/action/gripper_command.hpp>

GripperNode::GripperNode(const std::string &node_name)
    : Node(node_name)
{
    this->declare_parameter("gripper_action_name", "/gripper_controller/gripper_cmd");
    auto gripper_action_name = this->get_parameter("gripper_action_name").as_string();
    gripper_client_          = rclcpp_action::create_client<GripperCommand>(this, gripper_action_name);

    this->declare_parameter("gripper_close_joint_pos", 0.93);
    close_joint_pos_ = this->get_parameter("gripper_close_joint_pos").as_double();

    this->declare_parameter("gripper_open_joint_pos", 0.0);
    open_joint_pos_ = this->get_parameter("gripper_open_joint_pos").as_double();
}

void GripperNode::SendGripperCmd(double normalized_width, double max_effort)
{
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    if (!this->gripper_client_->wait_for_action_server(1s)) {
        logger_.Error("Action server not available after waiting");
        throw std::runtime_error("Action server not available after waiting");
    }

    if (normalized_width < 0.0 || normalized_width > 1.0) {
        auto msg = fmt::format("Normalized width must be between 0.0 and 1.0, got: {}", normalized_width);
        logger_.Error(msg);
        throw std::runtime_error(msg);
    }

    auto goal_msg               = GripperCommand::Goal();
    goal_msg.command.position   = NormalizedWidthToJointPos(normalized_width);
    goal_msg.command.max_effort = max_effort;

    auto send_goal_options                   = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GripperNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback      = std::bind(&GripperNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback        = std::bind(&GripperNode::result_callback, this, _1);

    this->gripper_client_->async_send_goal(goal_msg, send_goal_options);
}

void GripperNode::goal_response_callback(const GoalHandleCommand::SharedPtr &goal_handle)
{
    if (!goal_handle) {
        logger_.Error("Goal was rejected by server");
    } else {
        logger_.Info("Goal accepted by server, waiting for result");
    }
}

void GripperNode::feedback_callback(GoalHandleCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback)
{
    logger_.Info("Got feedback: pos: {}, effort: {}", feedback->position, feedback->effort);
}

void GripperNode::result_callback(const GoalHandleCommand::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            logger_.Info("Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            logger_.Error("Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            logger_.Warn("Goal was canceled");
            break;
        default:
            logger_.Error("Unknown result code");
            break;
    }
}
