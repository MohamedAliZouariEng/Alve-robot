#include "moveit2_scripts/get_pose_client.hpp"
#include "moveit2_scripts/robot_controller.hpp" // Include the RobotController class definition
#include <chrono>
#include <future>
#include <mutex>
#include <rclcpp/rclcpp.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
std::atomic<bool> poseReceived(false);


// Function prototypes remain the same but might be adapted or removed
// based on how you integrate them with RobotController
void setupPoseCallback(std::shared_ptr<GetPoseClient> &get_pose_client_node,std::mutex &pose_mutex, double &received_x,double &received_y, double &received_z);
void waitForPose(rclcpp::executors::MultiThreadedExecutor &executor,std::thread &executor_thread);
void cleanShutdown(rclcpp::executors::MultiThreadedExecutor &executor,std::thread &executor_thread);


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pick_and_place_node");

    // Initialize the RobotController with the appropriate planning groups
    RobotController robotController("arm_left", "hand_left");
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
    robotController.moveToHomePosition();

    double object_z_pick_center = received_z ;
    RCLCPP_INFO(LOGGER, "Z Pick pose: =%f", object_z_pick_center);
    robotController.moveToPregraspPositionLeft(received_x, received_y, received_z);
    robotController.moveToPickPositionLeft();
    cleanShutdown(executor, executor_thread);
    return 0;
}


// The implementations of setupPoseCallback, waitForPose, and cleanShutdown
// might remain largely unchanged but should be reviewed for integration with
// RobotController
void setupPoseCallback(std::shared_ptr<GetPoseClient> &get_pose_client_node,std::mutex &pose_mutex, double &received_x,double &received_y, double &received_z) {
    get_pose_client_node->register_pose_callback([&pose_mutex, &received_x, &received_y,&received_z](const GetPoseClient::PoseInfo &poses) {
        std::lock_guard<std::mutex> lock(pose_mutex);
        if (!poses.empty()) {
            received_x = poses.front().position.x - 0.3;
            received_y = poses.front().position.y;
            received_z = poses.front().position.z + 0.1;
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
void cleanShutdown(rclcpp::executors::MultiThreadedExecutor &executor, std::thread &executor_thread) {
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    rclcpp::shutdown();
}