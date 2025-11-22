#include "moveit2_scripts/get_pose_client.hpp"
#include "moveit2_scripts/robot_controller.hpp" // Include the RobotController class definition
#include <chrono>
#include <thread>
#include <future>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"


// Modify this function to include a timer
void publishMovementCommand(rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher,
                           rclcpp::Node::SharedPtr node,
                           double linear_x, double linear_y, double angular_z,
                           double duration_seconds) {
    auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    
    // Use the node's clock to get simulation time instead of system time
    twist_msg->header.stamp = node->now();
    twist_msg->header.frame_id = "base_footprint";
    twist_msg->twist.linear.x = linear_x;
    twist_msg->twist.linear.y = linear_y;
    twist_msg->twist.angular.z = angular_z;
    
    // Publish the movement command
    publisher->publish(*twist_msg);
    
    // Record start time using simulation clock
    auto start_time = node->now();
    auto target_duration = rclcpp::Duration::from_seconds(duration_seconds);
    
    // Wait for the specified duration using simulation time
    while (rclcpp::ok() && (node->now() - start_time) < target_duration) {
        // Sleep briefly to avoid busy waiting, but check simulation time frequently
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Stop the robot by publishing zero velocities
    twist_msg->twist.linear.x = 0.0;
    twist_msg->twist.linear.y = 0.0;
    twist_msg->twist.angular.z = 0.0;
    twist_msg->header.stamp = node->now();  // Use simulation time again
    publisher->publish(*twist_msg);
}




static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
std::atomic<bool> poseReceived(false);

rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr conveyor_subscriber_;
std::atomic<bool> forward_conveyor_flag_{false};


// Function prototypes remain the same but might be adapted or removed
// based on how you integrate them with RobotController
void setupPoseCallback(std::shared_ptr<GetPoseClient> &get_pose_client_node,std::mutex &pose_mutex, double &received_x,double &received_y, double &received_z);
void waitForPose(rclcpp::executors::MultiThreadedExecutor &executor,std::thread &executor_thread);
void cleanShutdown(rclcpp::executors::MultiThreadedExecutor &executor,
                  std::thread &executor_thread,
                  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr back_to_origin_publisher);


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pick_and_place_node");


    auto mecanum_publisher = node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mecanum_drive_controller/cmd_vel", 
        10
    );
    auto conveyor_subscriber = node->create_subscription<std_msgs::msg::Bool>(
    "/forward_conveyor",
    10,
    [&](const std_msgs::msg::Bool::SharedPtr msg) {
        forward_conveyor_flag_.store(msg->data);
        RCLCPP_INFO(LOGGER, "Forward conveyor received: %s", msg->data ? "True" : "False");
    });
    auto camera_view_publisher = node->create_publisher<std_msgs::msg::Bool>(
        "/camera_view_clear", 
        10
    );
    auto back_to_origin_publisher = node->create_publisher<std_msgs::msg::Bool>(
    "/back_to_origin", 
    10
);


    // Initialize the RobotController with the appropriate planning groups
    RobotController robotController("both_arms", "hand_right");
    robotController.setCameraViewPublisher(camera_view_publisher);
    auto get_pose_client_node = std::make_shared<GetPoseClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(robotController.node);
    executor.add_node(get_pose_client_node);
    double received_x = 0.0, received_y = 0.0, received_z = 0.0;
    std::mutex pose_mutex;
    setupPoseCallback(get_pose_client_node, pose_mutex, received_x, received_y,received_z);
    std::thread executor_thread([&]() { executor.spin(); });
    waitForPose(executor, executor_thread);
    //robotController.moveToHomePosition();

    double object_z_pick_center = received_z ;
    RCLCPP_INFO(LOGGER, "Z Pick pose: =%f", object_z_pick_center);
    robotController.moveToPregraspPositionRight(received_x, received_y, received_z);
    robotController.moveToPickPositionRight();
    //publishMovementCommand(mecanum_publisher, node, 0.0, -0.5, 0.0, 1.5); 
    //robotController.moveTheBoxDown();
    publishMovementCommand(mecanum_publisher, node, 0.0, 0.0, 1.5, 1.3); 
    publishMovementCommand(mecanum_publisher, node, 0.0, 0.5, 0.0, 1.0); 
    robotController.moveTheBoxDown();
    // Wait until forward_conveyor_flag_ becomes true
    while (!forward_conveyor_flag_.load()) {
        RCLCPP_INFO(LOGGER, "Waiting for forward_conveyor to become True...");
        rclcpp::sleep_for(std::chrono::milliseconds(500)); // Sleep to prevent busy waiting
    }

    RCLCPP_INFO(LOGGER, "Forward conveyor is True - Executing sequence");
        
    // Move forward for 3 seconds
    publishMovementCommand(mecanum_publisher, node, 0.0, 0.5, 0.0, 3.4);

    // Call placeTheBox method
    robotController.placeTheBox();

    // Move backward for 2 seconds
    publishMovementCommand(mecanum_publisher, node, 0.0, -0.5, 0.0, 2.0);

    // Rotate left for 6 seconds
    publishMovementCommand(mecanum_publisher, node, 0.0, 0.0, 1.5, 2.2);
    robotController.moveToHomePosition();
    cleanShutdown(executor, executor_thread, back_to_origin_publisher);
    return 0;
}


// The implementations of setupPoseCallback, waitForPose, and cleanShutdown
// might remain largely unchanged but should be reviewed for integration with
// RobotController
void setupPoseCallback(std::shared_ptr<GetPoseClient> &get_pose_client_node,std::mutex &pose_mutex, double &received_x,double &received_y, double &received_z) {
    get_pose_client_node->register_pose_callback([&pose_mutex, &received_x, &received_y,&received_z](const GetPoseClient::PoseInfo &poses) {
        std::lock_guard<std::mutex> lock(pose_mutex);
        if (!poses.empty()) {
            received_x = poses.front().position.x;
            received_y = poses.front().position.y ;
            received_z = poses.front().position.z;
            RCLCPP_INFO(LOGGER, "Pose received: X=%f Y=%f Z=%f", received_x, received_y, received_z);
            poseReceived.store(true); // Set the flag to true to indicate that the
            // pose has been received
        } 
        else {
            RCLCPP_INFO(LOGGER, "EMPTY POSES DETECTED");
        }
    });
}


void waitForPose(rclcpp::executors::MultiThreadedExecutor &executor, std::thread &executor_thread) {
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(10);

    while (!poseReceived.load() && std::chrono::steady_clock::now() - start < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to prevent busy waiting
        RCLCPP_INFO(LOGGER, "Waiting for Box Pose...");
    }

    if (!poseReceived.load()) {
        RCLCPP_ERROR(LOGGER, "Timeout waiting for pose. Exiting.");
        executor.cancel();
        if (executor_thread.joinable()) {
            executor_thread.join();
        }
        rclcpp::shutdown();
        exit(-1); // Exit due to timeout
    }
}
void cleanShutdown(rclcpp::executors::MultiThreadedExecutor &executor, std::thread &executor_thread,
                  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr back_to_origin_publisher) {

    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    back_to_origin_publisher->publish(msg);
    RCLCPP_INFO(LOGGER, "Published back_to_origin: true");
    
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    rclcpp::shutdown();
}
void conveyorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    forward_conveyor_flag_.store(msg->data);
    RCLCPP_INFO(LOGGER, "Forward conveyor received: %s", msg->data ? "True" : "False");
}