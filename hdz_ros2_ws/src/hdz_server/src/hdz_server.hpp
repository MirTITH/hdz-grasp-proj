#pragma once

#include <rclcpp/rclcpp.hpp>
#include "fmt_logger.hpp"
#include "fmt_user_types.hpp"
#include "moveit_node.hpp"
#include "gripper_node.hpp"

#include <hdz_server_msg/srv/move_to.hpp>
#include <hdz_server_msg/srv/move_to_named.hpp>
#include <hdz_server_msg/srv/get_end_effector_pose.hpp>
#include <hdz_server_msg/srv/get_joint_position.hpp>
#include <hdz_server_msg/srv/move_to_joint_position.hpp>
#include <hdz_server_msg/srv/set_gripper.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class HdzServer : public rclcpp::Node
{
    using MoveTo              = hdz_server_msg::srv::MoveTo;
    using MoveToNamed         = hdz_server_msg::srv::MoveToNamed;
    using GetEndEffectorPose  = hdz_server_msg::srv::GetEndEffectorPose;
    using GetJointPosition    = hdz_server_msg::srv::GetJointPosition;
    using MoveToJointPosition = hdz_server_msg::srv::MoveToJointPosition;
    using SetGripper          = hdz_server_msg::srv::SetGripper;

public:
    HdzServer(const std::string &node_name, std::shared_ptr<MoveItNode> moveit_node, std::shared_ptr<GripperNode> gripper_node)
        : Node(node_name), moveit_node_(std::move(moveit_node)), gripper_node_(std::move(gripper_node))
    {
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        move_to_service_                = this->create_service<MoveTo>("hdz_server/move_to", std::bind(&HdzServer::MoveToCallback, this, std::placeholders::_1, std::placeholders::_2));
        move_to_named_service_          = this->create_service<MoveToNamed>("hdz_server/move_to_named", std::bind(&HdzServer::MoveToNamedCallback, this, std::placeholders::_1, std::placeholders::_2));
        get_end_effector_pose_service_  = this->create_service<GetEndEffectorPose>("hdz_server/get_end_effector_pose", std::bind(&HdzServer::GetEndEffectorPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
        get_joint_position_service_     = this->create_service<GetJointPosition>("hdz_server/get_joint_position", std::bind(&HdzServer::GetJointPositionCallback, this, std::placeholders::_1, std::placeholders::_2));
        move_to_joint_position_service_ = this->create_service<MoveToJointPosition>("hdz_server/move_to_joint_position", std::bind(&HdzServer::MoveToJointPositionCallback, this, std::placeholders::_1, std::placeholders::_2));
        set_gripper_service_            = this->create_service<SetGripper>("hdz_server/set_gripper", std::bind(&HdzServer::SetGripperCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    FmtLogger logger_{this->get_logger()};
    std::shared_ptr<MoveItNode> moveit_node_;
    std::shared_ptr<GripperNode> gripper_node_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Service<MoveTo>::SharedPtr move_to_service_;
    rclcpp::Service<MoveToNamed>::SharedPtr move_to_named_service_;
    rclcpp::Service<GetEndEffectorPose>::SharedPtr get_end_effector_pose_service_;
    rclcpp::Service<GetJointPosition>::SharedPtr get_joint_position_service_;
    rclcpp::Service<MoveToJointPosition>::SharedPtr move_to_joint_position_service_;
    rclcpp::Service<SetGripper>::SharedPtr set_gripper_service_;

    geometry_msgs::msg::Pose TransformPose(const geometry_msgs::msg::PoseStamped original_pose, const std::string &target_frame_id)
    {
        // Get the transform from the request frame to the planning frame
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_->lookupTransform(target_frame_id, original_pose.header.frame_id, tf2::TimePointZero);

        // Transform the request frame to the planning frame
        geometry_msgs::msg::Pose target_pose;
        tf2::doTransform(original_pose.pose, target_pose, t);
        return target_pose;
    }

    void MoveToCallback(const std::shared_ptr<MoveTo::Request> request, std::shared_ptr<MoveTo::Response> response)
    {
        try {
            const auto frame_id            = request->pose_stamped.header.frame_id;
            const auto planning_frame_name = moveit_node_->move_group_->getPlanningFrame();

            logger_.Info("Received move_to request:");
            logger_.Info("    frame_id: {}, pose_stamped: {}", frame_id, request->pose_stamped);
            logger_.Info("    Planning frame: {}", planning_frame_name);

            geometry_msgs::msg::Pose target_pose = TransformPose(request->pose_stamped, planning_frame_name);

            moveit_node_->SetTargetPose(target_pose);
            moveit_node_->PlanAndMove();
        } catch (const std::exception &e) {
            response->status.is_success = false;
            response->status.message    = e.what();
        }
    }

    void MoveToNamedCallback(const std::shared_ptr<MoveToNamed::Request> request, std::shared_ptr<MoveToNamed::Response> response)
    {
        try {
            logger_.Info("Received move_to_named request: target: {}", request->named_target);

            moveit_node_->SetTargetByName(request->named_target);
            moveit_node_->PlanAndMove();
        } catch (const std::exception &e) {
            response->status.is_success = false;
            response->status.message    = e.what();
        }
    }

    void GetEndEffectorPoseCallback(const std::shared_ptr<GetEndEffectorPose::Request> request, std::shared_ptr<GetEndEffectorPose::Response> response)
    {
        try {
            logger_.Info("Received get_end_effector_pose request.");

            auto current_pose = moveit_node_->GetCurrentPose();

            response->pose_stamped.header.frame_id = request->frame_id;
            response->pose_stamped.header.stamp    = current_pose.header.stamp;
            response->pose_stamped.pose            = TransformPose(current_pose, request->frame_id);

            response->eef_name = moveit_node_->move_group_->getEndEffectorLink();
        } catch (const std::exception &e) {
            response->status.is_success = false;
            response->status.message    = e.what();
        }
    }

    void GetJointPositionCallback(const std::shared_ptr<GetJointPosition::Request> request, std::shared_ptr<GetJointPosition::Response> response)
    {
        (void)request;
        try {
            logger_.Info("Received get_joint_position request.");
            response->joint_position = moveit_node_->GetJointPos();
        } catch (const std::exception &e) {
            response->status.is_success = false;
            response->status.message    = e.what();
        }
    }

    void MoveToJointPositionCallback(const std::shared_ptr<MoveToJointPosition::Request> request, std::shared_ptr<MoveToJointPosition::Response> response)
    {
        try {
            logger_.Info("Received move_to_joint_position request.");

            auto expected_size = moveit_node_->GetJointPos().size();
            if (request->joint_position.size() != expected_size) {
                auto msg = fmt::format("Joint position size mismatch. Expected: {}, Got: {}", expected_size, request->joint_position.size());
                throw std::runtime_error(msg);
            }

            moveit_node_->move_group_->setJointValueTarget(request->joint_position);
            moveit_node_->PlanAndMove();
        } catch (const std::exception &e) {
            response->status.is_success = false;
            response->status.message    = e.what();
        }
    }

    void SetGripperCallback(const std::shared_ptr<SetGripper::Request> request, std::shared_ptr<SetGripper::Response> response)
    {
        try {
            logger_.Info("Received set_gripper request: {}", request->normalized_width);

            gripper_node_->SendGripperCmd(request->normalized_width, request->max_effort);
        } catch (const std::exception &e) {
            response->status.is_success = false;
            response->status.message    = e.what();
        }
    }
};