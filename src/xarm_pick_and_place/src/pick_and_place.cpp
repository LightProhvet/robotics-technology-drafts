#include <memory>
#include <cmath>
#include <chrono>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.h>

// Joint positions in radians for xarm7 - i chose what i like through rviz
// Location A: "home-like" pose — 0, 0, 0, 90, 0, 85, 0 degrees
static const std::vector<double> LOCATION_A = {
    0.0, 0.0, 0.0, M_PI_2, 0.0, 1.4835, 0.0
};

// Location B: place pose — -14, -66, -74, 115, -84, 108, -118 degrees
static const std::vector<double> LOCATION_B = {
    -0.2443, -1.1519, -1.2915, 2.0071, -1.4661, 1.8850, -2.0594
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("pick_and_place");
    auto const logger = rclcpp::get_logger("pick_and_place");

    using moveit::planning_interface::MoveGroupInterface;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spin_until_fresh = [&]() {
        executor.spin_some(std::chrono::milliseconds(200));
    };

    auto arm = MoveGroupInterface(node, "xarm7");
    auto gripper = MoveGroupInterface(node, "xarm_gripper");

    auto abort = [&](const std::string& reason) {
        RCLCPP_ERROR(logger, "ABORTING: %s", reason.c_str());
        rclcpp::shutdown();
        exit(1);
    };

    // Move relative to a known joint configuration using FK (avoids stale getCurrentPose)
    auto move_cartesian = [&](const std::vector<double>& from_joints, double dx, double dy, double dz, const std::string& label = "") {
        const moveit::core::JointModelGroup* jmg =
            arm.getRobotModel()->getJointModelGroup("xarm7");

        auto state = arm.getCurrentState();
        state->setJointGroupPositions(jmg, from_joints);
        state->updateLinkTransforms();
        const Eigen::Isometry3d& eef_tf = state->getGlobalLinkTransform(arm.getEndEffectorLink());

        geometry_msgs::msg::Pose target;
        target.position.x = eef_tf.translation().x() + dx;
        target.position.y = eef_tf.translation().y() + dy;
        target.position.z = eef_tf.translation().z() + dz;
        Eigen::Quaterniond q(eef_tf.rotation());
        target.orientation.x = q.x();
        target.orientation.y = q.y();
        target.orientation.z = q.z();
        target.orientation.w = q.w();

        arm.setStartState(*state);

        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        if (!label.empty()) RCLCPP_INFO(logger, "%s (%.0f%% achieved)", label.c_str(), fraction * 100.0);
        if (fraction > 0.9) {
            MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
                abort("execution failed: " + label);
        } else {
            abort("cartesian path planning failed (<90%): " + label);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));

    };

    auto move_to_joints = [&](const std::vector<double>& joints, const std::string& label = "") {
        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(joints);
        if (!label.empty()) RCLCPP_INFO(logger, "Planning: %s", label.c_str());
        MoveGroupInterface::Plan plan;
        if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            abort("planning failed: " + label);
        if (!label.empty()) RCLCPP_INFO(logger, "Executing: %s", label.c_str());
        if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            abort("execution failed: " + label);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    };

    auto open_gripper = [&]() {
        gripper.setNamedTarget("open");
        MoveGroupInterface::Plan plan;
        if (gripper.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            abort("gripper open planning failed");
        if (gripper.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            abort("gripper open execution failed");
    };

    auto close_gripper = [&]() {
        gripper.setNamedTarget("close");
        MoveGroupInterface::Plan plan;
        if (gripper.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            abort("gripper close planning failed");
        if (gripper.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            abort("gripper close execution failed");
    };

    auto log_poses = [&](const std::string& tag) {
        auto state = arm.getCurrentState();
        auto planning_frame = arm.getPlanningFrame();
        auto eef_link = arm.getEndEffectorLink();
        auto pose = arm.getCurrentPose().pose;

        std::stringstream ss;
        state->printStatePositions(ss);
        RCLCPP_INFO(logger, "[%s] state:\n%s", tag.c_str(), ss.str().c_str());
        RCLCPP_INFO(logger, "[%s] planning_frame=%s  eef_link=%s",
            tag.c_str(), planning_frame.c_str(), eef_link.c_str());
        RCLCPP_INFO(logger, "[%s] getCurrentPose: x=%.4f y=%.4f z=%.4f | qx=%.4f qy=%.4f qz=%.4f qw=%.4f",
            tag.c_str(),
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    };
    (void)log_poses;  // suppress unused warning

    // --- Pick and Place sequence ---

    // 1. Move to location A
    move_to_joints(LOCATION_A, "Moving to location A");

    // 2. Open gripper
    RCLCPP_INFO(logger, "Opening gripper");
    open_gripper();

    // 3. Lower to grasp
    move_cartesian(LOCATION_A, 0.0, 0.0, -0.2, "Lowering to grasp");

    // 4. Close gripper (pick)
    RCLCPP_INFO(logger, "Closing gripper");
    close_gripper();

    // 5. Move up before joint move
    //move_cartesian(LOCATION_A, 0.0, 0.0, 0.2, "Raising after pick");

    // 6. Move to location B
    move_to_joints(LOCATION_B, "Moving to location B");

    // 7. Open gripper (place)
    RCLCPP_INFO(logger, "Opening gripper (place)");
    open_gripper();

    RCLCPP_INFO(logger, "Pick and place complete!");

    rclcpp::shutdown();
    return 0;
}
