#include "moveit2_scripts/robot_controller.hpp"
#include <chrono>
#include <thread>

RobotController::RobotController(const std::string &planning_group, const std::string &planning_group_gripper): node(rclcpp::Node::make_shared("move_group_demo")),move_group(node, planning_group),move_group_gripper(node, planning_group_gripper) {
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    move_group.setPlanningTime(10.0);
    move_group.setPoseReferenceFrame("base_footprint");
}
void RobotController::setCameraViewPublisher(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher) {
    camera_view_publisher_ = publisher;
}

void RobotController::moveToHomePosition() {
    std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group.setJointValueTarget(home_position);
    executeMovement(move_group, "Moved to home position successfully.", "Failed to move to home position.");
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

void RobotController::moveToPickPositionRight() {
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    current_joint_values[1] -= 0.15; // Adjust first joint
    current_joint_values[8] += 0.15;
    move_group.setJointValueTarget(current_joint_values);
    executeMovement(move_group, "Moved to pick position successfully.", "Failed to move to pick position.");
    
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(2));


    /*
    current_joint_values[2] -= 0.18; // Adjust first joint
    current_joint_values[9] += 0.18;
    move_group.setJointValueTarget(current_joint_values);
    executeMovement(move_group, "Moved to pick position right successfully.", "Failed to move to pick position right.");
    
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(4));
    
    current_joint_values = move_group.getCurrentJointValues();
    current_joint_values[7] -= 0.10;
    current_joint_values[0] += 0.10;
    move_group.setJointValueTarget(current_joint_values);
    executeMovement(move_group, "Moved to pick position right successfully.", "Failed to move to pick position right.");
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
*/
}



void RobotController::moveTheBoxDown() {
    bool success = false;
    
    try {
        std::vector<double> current_joint_values = move_group.getCurrentJointValues();
        current_joint_values[7] += 0.60; // Adjust first joint
        current_joint_values[0] -= 0.60;
        move_group.setJointValueTarget(current_joint_values);
        
        // Execute movement and capture success
        if (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Moved to pick position right successfully.");
            success = true;
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to move to pick position right.");
            success = false;
        }
        
        RCLCPP_INFO(node->get_logger(), "Waiting for 4 seconds...");
        std::this_thread::sleep_for(std::chrono::seconds(4));
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in moveTheBoxDown: %s", e.what());
        success = false;
    }
    
    // Publish the camera view clear status
    if (camera_view_publisher_) {
        auto msg = std_msgs::msg::Bool();
        msg.data = success;
        camera_view_publisher_->publish(msg);
        RCLCPP_INFO(node->get_logger(), "Published camera_view_clear: %s", success ? "True" : "False");
    } else {
        RCLCPP_WARN(node->get_logger(), "Camera view publisher not set!");
    }
}


void RobotController::placeTheBox() {
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    current_joint_values[7] -= 0.60; // Adjust first joint
    current_joint_values[0] += 0.60;
    move_group.setJointValueTarget(current_joint_values);
    executeMovement(move_group, " place the box successfully.", "Failed to move to place the box.");
    
    RCLCPP_INFO(node->get_logger(), "Waiting for 4 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(4));


    current_joint_values[2] += 0.30; // Adjust first joint
    current_joint_values[9] -= 0.30;
    move_group.setJointValueTarget(current_joint_values);
    executeMovement(move_group, "Moved to pick position right successfully.", "Failed to move to pick position right.");
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

}

void RobotController::moveToPregraspPositionRight(double x_pregrasp,double y_pregrasp,double z_pregrasp) {
    geometry_msgs::msg::Pose target_pose_right;
    geometry_msgs::msg::Pose target_pose_left;

    target_pose_right.position.x = x_pregrasp + 0.23;
    target_pose_right.position.y = y_pregrasp + 0.07;
    target_pose_right.position.z = z_pregrasp;
    target_pose_right.orientation.x = 0.609;  // Quaternion x
    target_pose_right.orientation.y = -0.579; // Quaternion y
    target_pose_right.orientation.z = -0.530; // Quaternion z
    target_pose_right.orientation.w = 0.114;

    target_pose_left.position.x = x_pregrasp - 0.3;
    target_pose_left.position.y = y_pregrasp + 0.07;
    target_pose_left.position.z = z_pregrasp ;
    target_pose_left.orientation.x = 0.531;  // Quaternion x
    target_pose_left.orientation.y = -0.116; // Quaternion y
    target_pose_left.orientation.z = -0.606; // Quaternion z
    target_pose_left.orientation.w = 0.580;
    move_group.setPoseTarget(target_pose_right, "R_link_7");
    move_group.setPoseTarget(target_pose_left, "L_link_7");

    executeMovement(move_group, "Moved to pregrasp position successfully.", "Failed to move to pregrasp position.");
}



void RobotController::executeMovement(moveit::planning_interface::MoveGroupInterface &group,const std::string &success_msg, const std::string &fail_msg) {
    if (group.move() == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "%s", success_msg.c_str());
    } 
    else {
        RCLCPP_ERROR(node->get_logger(), "%s", fail_msg.c_str());
    }
}