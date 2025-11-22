#include "moveit2_scripts/get_pose_client.hpp"
#include "moveit2_scripts/robot_controller.hpp"
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

struct ArmPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    bool received = false;
    std::mutex mutex;
};

void setupPoseCallbacks(std::shared_ptr<GetPoseClient> get_pose_client_node, 
                       ArmPose& left_pose, ArmPose& right_pose) {
    get_pose_client_node->register_pose_callback([&](const GetPoseClient::PoseInfo &poses) {
        if (poses.empty()) {
            RCLCPP_INFO(LOGGER, "No poses detected");
            return;
        }

        auto pose = poses.front();
        
        // Left arm pose
        {
            std::lock_guard<std::mutex> lock(left_pose.mutex);
            if (!left_pose.received) {
                left_pose.x = pose.position.x - 0.3;
                left_pose.y = pose.position.y;
                left_pose.z = pose.position.z + 0.1;
                left_pose.received = true;
                RCLCPP_INFO(LOGGER, "Left Pose: X=%.3f Y=%.3f Z=%.3f", 
                           left_pose.x, left_pose.y, left_pose.z);
            }
        }
        
        // Right arm pose
        {
            std::lock_guard<std::mutex> lock(right_pose.mutex);
            if (!right_pose.received) {
                right_pose.x = pose.position.x + 0.2;
                right_pose.y = pose.position.y;
                right_pose.z = pose.position.z;
                right_pose.received = true;
                RCLCPP_INFO(LOGGER, "Right Pose: X=%.3f Y=%.3f Z=%.3f", 
                           right_pose.x, right_pose.y, right_pose.z);
            }
        }
    });
}

void executeArmOperations(RobotController& left_controller, RobotController& right_controller,
                         ArmPose& left_pose, ArmPose& right_pose) {
    // Wait for both poses to be received
    auto start = std::chrono::steady_clock::now();
    while (true) {
        {
            std::lock_guard<std::mutex> lock_left(left_pose.mutex);
            std::lock_guard<std::mutex> lock_right(right_pose.mutex);
            if (left_pose.received && right_pose.received) {
                break;
            }
        }

        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) {
            RCLCPP_ERROR(LOGGER, "Timeout waiting for poses");
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Execute left arm operation
    try {
        RCLCPP_INFO(LOGGER, "Executing left arm movement");
        left_controller.moveToHomePosition();
        left_controller.moveToPregraspPositionLeft(left_pose.x, left_pose.y, left_pose.z);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Left arm error: %s", e.what());
    }

    // Execute right arm operation
    try {
        RCLCPP_INFO(LOGGER, "Executing right arm movement");
        right_controller.moveToHomePosition();
        right_controller.moveToPregraspPositionRight(right_pose.x, right_pose.y, right_pose.z);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Right arm error: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // Create nodes
    auto node = std::make_shared<rclcpp::Node>("pick_and_place_node");
    RobotController left_controller("arm_left", "hand_left");
    RobotController right_controller("arm_right", "hand_right");
    auto get_pose_client = std::make_shared<GetPoseClient>();

    // Setup pose detection
    ArmPose left_pose, right_pose;
    setupPoseCallbacks(get_pose_client, left_pose, right_pose);

    // Start ROS executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(left_controller.node);
    executor.add_node(right_controller.node);
    executor.add_node(get_pose_client);
    
    std::thread executor_thread([&]() { executor.spin(); });

    // Execute operations
    executeArmOperations(left_controller, right_controller, left_pose, right_pose);

    // Cleanup
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    rclcpp::shutdown();
    
    return 0;
}