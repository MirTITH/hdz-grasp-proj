#include "hdz_grpc_move_grasp.hpp"
#include "fmt_logger.hpp"
#include <fmt_user_types.hpp>

static geometry_msgs::msg::Pose ConvertToPose(const hdz_grpc_msg::Pose &pose)
{
    geometry_msgs::msg::Pose p;
    p.position.x    = pose.position().x();
    p.position.y    = pose.position().y();
    p.position.z    = pose.position().z();
    p.orientation.x = pose.orientation().x();
    p.orientation.y = pose.orientation().y();
    p.orientation.z = pose.orientation().z();
    p.orientation.w = pose.orientation().w();
    return p;
}

grpc::Status MoveGraspServiceImpl::MoveTo(::grpc::ServerContext *context, const ::hdz_grpc_msg::PoseStamped *request, ::hdz_grpc_msg::BoolWithMsg *response)
{
    (void)context;
    kLogger.Info("MoveTo called. frame_name: {}, pose: {}", request->frame_name(), request->pose());
    const auto frame_name               = request->frame_name();
    const geometry_msgs::msg::Pose pose = ConvertToPose(request->pose());

    auto planning_frame_name = moveit_node_->move_group_->getPlanningFrame().c_str();
    kLogger.Info("Planning frame: {}", planning_frame_name);

    // Get the transform from the request frame to the planning frame
    geometry_msgs::msg::TransformStamped t;
    try {
        t = moveit_node_->tf_buffer_->lookupTransform(planning_frame_name, frame_name, tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
        response->set_is_success(false);
        response->set_msg(e.what());
        kLogger.Info("Could not transform %s to %s: %s", frame_name, planning_frame_name, e.what());
        return grpc::Status::OK;
    }

    // Transform the request frame to the planning frame
    geometry_msgs::msg::Pose target_pose;
    tf2::doTransform(pose, target_pose, t);
    kLogger.Info("Transformed pose: {}", target_pose);

    try {
        moveit_node_->SetTargetPose(target_pose);
        moveit_node_->PlanAndMove();
    } catch (const std::exception &e) {
        response->set_is_success(false);
        response->set_msg(e.what());
        return grpc::Status::OK;
    }

    response->set_is_success(true);
    return grpc::Status::OK;
}

grpc::Status MoveGraspServiceImpl::MoveToNamed(::grpc::ServerContext *context, const ::hdz_grpc_msg::StrMsg *request, ::hdz_grpc_msg::BoolWithMsg *response)
{
    (void)context;
    kLogger.Info("Received move_to_named request:");
    kLogger.Info("    target: {}", request->str());

    try {
        moveit_node_->SetTargetByName(request->str());
        moveit_node_->PlanAndMove();
    } catch (const std::exception &e) {
        response->set_is_success(false);
        response->set_msg(e.what());
        return grpc::Status::OK;
    }

    response->set_is_success(true);
    return grpc::Status::OK;
}
grpc::Status MoveGraspServiceImpl::MoveToJointPosition(::grpc::ServerContext *context, const ::hdz_grpc_msg::Array *request, ::hdz_grpc_msg::BoolWithMsg *response)
{
    (void)context;
    kLogger.Info("Received move_to_joint_position request:");

    auto joint_group_positions = moveit_node_->GetJointPos();
    if (joint_group_positions.size() != (size_t)request->data().size()) {
        response->set_is_success(false);
        response->set_msg("Invalid joint position size");
        return grpc::Status::OK;
    }

    for (size_t i = 0; i < joint_group_positions.size(); ++i) {
        joint_group_positions[i] = request->data(i);
    }

    moveit_node_->move_group_->setJointValueTarget(joint_group_positions);

    try {
        moveit_node_->PlanAndMove();
    } catch (const std::exception &e) {
        response->set_is_success(false);
        response->set_msg(e.what());
        return grpc::Status::OK;
    }

    response->set_is_success(true);
    return grpc::Status::OK;
}

grpc::Status MoveGraspServiceImpl::SetGripper(::grpc::ServerContext *context, const ::hdz_grpc_msg::SetGripperRequest *request, ::hdz_grpc_msg::BoolWithMsg *response)
{
    (void)context;
}