#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include "std_msgs/msg/bool.hpp"


class RobotController {
    public:

        // Constructor declaration
        RobotController(const std::string &planning_group,const std::string &planning_group_gripper);

        // Public member functions declaration
        void moveToHomePosition();
        void moveToPickPositionRight();
        void moveToPickPositionLeft();
        void moveTheBoxDown();
        void placeTheBox();
        void moveToPregraspPositionRight(double x_pregrasp, double y_pregrasp,double z_pregrasp = 0.284);
        void moveToPregraspPositionLeft(double x_pregrasp, double y_pregrasp,double z_pregrasp = 0.284);

        void setCameraViewPublisher(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher);

        // Publicly accessible node for adding to executor
        rclcpp::Node::SharedPtr node;

    private:

        // Private member variables declaration
        moveit::planning_interface::MoveGroupInterface move_group;
        
        moveit::planning_interface::MoveGroupInterface move_group_gripper;

        // Private member functions declaration
        void executeMovement(moveit::planning_interface::MoveGroupInterface &group,const std::string &success_msg,const std::string &fail_msg);
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_view_publisher_;
        
};
#endif // ROBOT_CONTROLLER_HPP