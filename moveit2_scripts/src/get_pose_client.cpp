#include "moveit2_scripts/get_pose_client.hpp"
#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>


GetPoseClient::GetPoseClient(const rclcpp::NodeOptions &node_options): Node("get_pose_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<Find>(this->get_node_base_interface(), this->get_node_graph_interface(),this->get_node_logging_interface(), this->get_node_waitables_interface(),"find_objects");
    this->timer_ =this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&GetPoseClient::send_goal, this));
}

bool GetPoseClient::is_goal_done() const { return goal_done_; }



void GetPoseClient::send_goal() {
    using namespace std::placeholders;
    this->timer_->cancel();
    goal_done_ = false;

    if (!this->client_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(),"Action server not available after waiting");
        goal_done_ = true;
        return;
    }

    auto goal_msg = Find::Goal();
    goal_msg.plan_grasps = false;
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GetPoseClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&GetPoseClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&GetPoseClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}


void GetPoseClient::register_pose_callback(GetPoseClient::PoseCallback callback) {
    pose_callback_ = callback;
}


void GetPoseClient::goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } 
    else {
        RCLCPP_INFO(this->get_logger(),"Goal accepted by server, waiting for result");
    }
}


void GetPoseClient::feedback_callback(GoalHandleFind::SharedPtr,const std::shared_ptr<const Find::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
}


void GetPoseClient::result_callback(const GoalHandleFind::WrappedResult &result) {
    this->goal_done_ = true;
    
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(this->get_logger(), "Action failed: %d", static_cast<int>(result.code));
        return;
    }

    // Check if we have any objects
    if (result.result->objects.empty()) {
        RCLCPP_WARN(this->get_logger(), "No objects detected");
        return;
    }

    // Get the frame_id from the first object's header
    std::string frame_id = result.result->objects[0].object.header.frame_id;
    RCLCPP_INFO(this->get_logger(), "Processing %zu objects in frame: %s", 
        result.result->objects.size(),
        frame_id.c_str());

    last_pose_info_.clear();
    for (const auto& graspable_object : result.result->objects) {
        const auto& object = graspable_object.object;
        
        // Basic validation
        if (object.primitives.empty() || object.primitive_poses.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Object missing primitive data");
            continue;
        }

        // Get the first primitive and pose (assuming simple objects)
        const auto& dim = object.primitives[0].dimensions;
        const auto& pose = object.primitive_poses[0];

        // Log detailed object information
        RCLCPP_INFO(this->get_logger(),
            "Object '%s' in frame %s - Type: %d, Dim: %.3fx%.3fx%.3f, Pose: (%.3f,%.3f,%.3f)",
            object.name.c_str(),
            object.header.frame_id.c_str(),
            object.primitives[0].type,
            dim[0], dim[1], dim[2],
            pose.position.x, pose.position.y, pose.position.z);

        // Filter for boxes of reasonable size
        if (object.primitives[0].type == shape_msgs::msg::SolidPrimitive::BOX) {
            const float MAX_DIMENSION = 1.0f; // 1 meter maximum
            if (dim.size() >= 3 && 
                dim[0] < MAX_DIMENSION && 
                dim[1] < MAX_DIMENSION && 
                dim[2] < MAX_DIMENSION) {
                last_pose_info_.push_back(pose);
            }
        }
    }

    if (pose_callback_) {
        pose_callback_(last_pose_info_);
    }
}