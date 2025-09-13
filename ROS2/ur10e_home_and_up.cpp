#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

class UR10eHomeAndUp : public rclcpp::Node
{
public:
    UR10eHomeAndUp() : Node("ur10e_home_and_up")
    {
        RCLCPP_INFO(this->get_logger(), "Node created. Ready to run...");
    }

    void run()
    {
        moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "ur_manipulator");

        move_group.setPlanningTime(15.0);
        move_group.setMaxVelocityScalingFactor(0.2);
        move_group.setMaxAccelerationScalingFactor(0.2);

        // --- Home position ---
        std::vector<double> home_joints = move_group.getCurrentJointValues();
        home_joints[0] = 0.4001;   // shoulder_pan_joint
        home_joints[1] = -0.7911;  // shoulder_lift_joint
        home_joints[2] = 1.8296;   // elbow_joint
        home_joints[3] = -2.6102;  // wrist_1_joint
        home_joints[4] = 1.5754;   // wrist_2_joint
        // giữ nguyên wrist_3_joint

        move_group.setJointValueTarget(home_joints);

        moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        if (move_group.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Moving to home...");
            move_group.execute(home_plan);
            RCLCPP_INFO(this->get_logger(), "Reached home position.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to home!");
            return;
        }

        // --- Ask user to move along Z ---
        geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "Current pose: x=%.3f y=%.3f z=%.3f",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z);

        std::cout << "Do you want to move 8.55 cm along Z? (y/n): ";   // 9.1 cm maximum
        char ans_z;
        std::cin >> ans_z;

        if (ans_z == 'y' || ans_z == 'Y')
        {
            geometry_msgs::msg::Pose target_pose = current_pose.pose;
            target_pose.position.z += 0.0855; // move up 8.55 cm

            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose);

            moveit_msgs::msg::RobotTrajectory traj;
            double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, traj);

            if (fraction > 0.99)
            {
                RCLCPP_INFO(this->get_logger(), "Executing Cartesian path up 2cm...");
                move_group.execute(traj);
                RCLCPP_INFO(this->get_logger(), "Reached target Z.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Z path failed (fraction=%.2f)", fraction);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR10eHomeAndUp>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // Run node in separate thread
    std::thread run_thread([node]() { node->run(); });
    exec.spin();
    run_thread.join();

    rclcpp::shutdown();
    return 0;
}
