// GetPoseClient.h
#ifndef GETPOSECLIENT_H
#define GETPOSECLIENT_H

#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <functional>
#include <memory>
#include <vector>

class GetPoseClient : public rclcpp::Node {
    public:

        using Find = grasping_msgs::action::FindGraspableObjects;
        using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;
        using PoseInfo = std::vector<geometry_msgs::msg::Pose>; // Assuming Pose type from ROS2

        // geometry_msgs
        using PoseCallback = std::function<void(const PoseInfo &)>;
        explicit GetPoseClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
        bool is_goal_done() const;
        void send_goal();
        void register_pose_callback(PoseCallback callback);

    private:

        rclcpp_action::Client<Find>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool goal_done_;
        PoseInfo last_pose_info_;
        PoseCallback pose_callback_;
        void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle);
        void feedback_callback(GoalHandleFind::SharedPtr, const std::shared_ptr<const Find::Feedback> feedback);
        void result_callback(const GoalHandleFind::WrappedResult &result);
};
#endif // GETPOSECLIENT_H