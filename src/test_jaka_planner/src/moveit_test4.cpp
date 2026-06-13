#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;

void sigintHandler(int /*sig*/)
{
    rclcpp::shutdown();
}

class LinearMotionPlanner : public rclcpp::Node
{
public:
    LinearMotionPlanner() : Node("linear_motion_planner")
    {
        // Declare parameters
        model_ = this->declare_parameter<string>("model", "zu3");
        planning_time_ = this->declare_parameter<double>("planning_time", 10.0);
    }

    void initializeMoveGroup()
    {
        // Initialize MoveGroupInterface
        string PLANNING_GROUP = "jaka_" + model_;
        RCLCPP_INFO(this->get_logger(), "Using PLANNING_GROUP: %s", PLANNING_GROUP.c_str());
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), PLANNING_GROUP);
        move_group_->setPlanningTime(planning_time_);
    }
    void moveToInitialPosition()
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = -0.35;
        target_pose.position.y = 0.01;
        target_pose.position.z = 0.130;

        tf2::Quaternion quaternion;
        quaternion.setRPY(M_PI, 0.0, 0.0);
        target_pose.orientation.x = quaternion.x();
        target_pose.orientation.y = quaternion.y();
        target_pose.orientation.z = quaternion.z();
        target_pose.orientation.w = quaternion.w();

        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "Initial position planning success: %s", success ? "True" : "False");

        if (success)
        {
            move_group_->move();
            RCLCPP_INFO(this->get_logger(), "Moved to initial position.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to initial position.");
        }
    }
    void moveLinear(double dx, double dy, double dz)
    {
        // Get the current pose of the end effector
        // geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
        // auto current_pose = move_group_->getCurrentPose().pose;
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

        current_pose.pose.position.y += dy;
        current_pose.pose.position.x += dx;
        current_pose.pose.position.z += dz;
        move_group_->setPoseTarget(current_pose.pose);

        // 规划并执行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            move_group_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Linear motion executed successfully.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan linear motion.");
            return;
        }
    }

private:
    string model_;
    double planning_time_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);

    auto node = std::make_shared<LinearMotionPlanner>();
    node->initializeMoveGroup();   // 初始化 MoveGroupInterface
    node->moveToInitialPosition(); // Move to initial position
    // Example: Move 0.1m along x-axis, 0.05m along y-axis, and 0.02m along z-axis
    node->moveLinear(0.1, 0.0, 0.0);  // Move along x-axis
    node->moveLinear(0.0, 0.05, 0.0); // Move along y-axis
    node->moveLinear(0.0, 0.0, 0.02); // Move along z-axis

    rclcpp::shutdown();
    return 0;
}