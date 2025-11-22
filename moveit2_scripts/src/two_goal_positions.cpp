#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <chrono>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "arm_right";


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);


    // We set parameters
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setPlanningTime(10.0);
    move_group.setPoseReferenceFrame("base_footprint");
    const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


    // Go To Position 1
    std::vector<double> pos1 = {-1.5527, 0.0, 0.0, 0.1995, 0.0, 0.0, 0.0};
    move_group.setJointValueTarget(pos1);
    // Execute the trajectory
    if (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Moved to pose 1 executed successfully.");
    } 
    else {
        RCLCPP_ERROR(LOGGER, "Failed to execute pose 1.");
    }

    // Add 2 second delay between pose1 and pose2
    RCLCPP_INFO(LOGGER, "Waiting for 2 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(2));


    // Go To Position 2
    std::vector<double> pos2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group.setJointValueTarget(pos2);
    // Execute the trajectory
    if (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Moved to pose 2 executed successfully.");
    } 
    else {
        RCLCPP_ERROR(LOGGER, "Failed to execute pose 2.");
    }
    rclcpp::shutdown();
    return 0;
}