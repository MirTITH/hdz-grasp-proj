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
    } catch (const std::exception &e) {
        response->set_is_success(false);
        response->set_msg(e.what());
        return grpc::Status::OK;
    }

    try {
        moveit_node_->PlanAndMove();
    } catch (const std::exception &e) {
        response->set_is_success(false);
        response->set_msg(e.what());
        return grpc::Status::OK;
    }

    return grpc::Status::OK;
}
