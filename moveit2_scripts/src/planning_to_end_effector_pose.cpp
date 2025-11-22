#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    static const std::string PLANNING_GROUP = "arm_right";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    
    // We set parameters
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setPlanningTime(10.0);
    move_group.setPoseReferenceFrame("base_footprint");
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.246;
    target_pose1.position.y = 0.492;
    target_pose1.position.z = 0.865;

    // Set orientation if necessary
    target_pose1.orientation.x = 0.609;  // Quaternion x
    target_pose1.orientation.y = -0.579; // Quaternion y
    target_pose1.orientation.z = -0.530; // Quaternion z
    target_pose1.orientation.w = 0.114;
    move_group.setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Moved to End Effector pose successfully.");
    } else {
    RCLCPP_ERROR(LOGGER, "Failed to move to End Effector pose.");
    return 1;
    }

    // Execute the trajectory
    if (move_group.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Moved to End Effector pose executed successfully.");
    } 
    else {
        RCLCPP_ERROR(LOGGER, "Failed to move to End Effector pose.");
    }

    rclcpp::shutdown();
    return 0;
}