#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;

void sigintHandler(int /*sig*/)
{
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    signal(SIGINT, sigintHandler);
    auto node = rclcpp::Node::make_shared("jaka_planner", options);

    // Declare the parameter for robot model with a default value of "zu3"
    string model = node->declare_parameter<string>("model", "zu3");

    // Declare parameters for target pose
    double x_position = node->declare_parameter<double>("x_position", -0.35); // Default in m
    double y_position = node->declare_parameter<double>("y_position", 0.01);  // Default in m
    double z_position = node->declare_parameter<double>("z_position", 0.130); // Default in m
    double rx = node->declare_parameter<double>("rx", M_PI);                  // Default in radians
    double ry = node->declare_parameter<double>("ry", 0.0);                   // Default in radians
    double rz = node->declare_parameter<double>("rz", 0.0);                   // Default in radians
    bool use_metric = node->declare_parameter<bool>("use_metric", true);      // Default to metric (meters and radians)

    // Convert units if necessary
    if (!use_metric)
    {
        // Convert mm to meters
        x_position /= 1000.0;
        y_position /= 1000.0;
        z_position /= 1000.0;

        // Convert degrees to radians
        rx = rx * M_PI / 180.0;
        ry = ry * M_PI / 180.0;
        rz = rz * M_PI / 180.0;
    }

    // Automatically construct the PLANNING_GROUP by concatenating "jaka_" + model
    string PLANNING_GROUP = "jaka_" + model;
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Using PLANNING_GROUP: %s", PLANNING_GROUP.c_str());

    // Create MoveGroupInterface for controlling the robot
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Set planning time to handle timeout issues
    move_group.setPlanningTime(10.0); // Set timeout to 10 seconds

    // Define target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x_position;
    target_pose.position.y = y_position;
    target_pose.position.z = z_position;

    // Convert roll, pitch, yaw to quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(rx, ry, rz);
    target_pose.orientation.x = quaternion.x();
    target_pose.orientation.y = quaternion.y();
    target_pose.orientation.z = quaternion.z();
    target_pose.orientation.w = quaternion.w();

    // Set target pose
    move_group.setPoseTarget(target_pose);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Planning success: %s", success ? "True" : "False");

    if (success)
    {
        move_group.move(); // Execute the motion
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("jaka_planner"), "Failed to plan to the target pose.");
    }

    rclcpp::shutdown();
    return 0;
}