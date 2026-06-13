#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "custom_msgs/srv/move_to_point.hpp"
using namespace std;

void sigintHandler(int /*sig*/)
{
    rclcpp::shutdown();
}

class JakaPlanner : public rclcpp::Node
{
public:
    JakaPlanner() : Node("jaka_planner")
    {
        // Declare parameters
        model_ = this->declare_parameter<string>("model", "zu3");
        x_position_ = this->declare_parameter<double>("x_position", -0.35);
        y_position_ = this->declare_parameter<double>("y_position", 0.01);
        z_position_ = this->declare_parameter<double>("z_position", 0.130);
        rx_ = this->declare_parameter<double>("rx", M_PI);
        ry_ = this->declare_parameter<double>("ry", 0.0);
        rz_ = this->declare_parameter<double>("rz", 0.0);

        // Create service to handle new target positions
        service_ = this->create_service<custom_msgs::srv::MoveToPoint>(
            "move_to_point",
            std::bind(&JakaPlanner::handleMoveRequest, this, std::placeholders::_1, std::placeholders::_2));
    }

    void initializeMoveGroup()
    {
        // Initialize MoveGroupInterface
        string PLANNING_GROUP = "jaka_" + model_;
        RCLCPP_INFO(this->get_logger(), "Using PLANNING_GROUP: %s", PLANNING_GROUP.c_str());
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), PLANNING_GROUP);
        move_group_->setPlanningTime(10.0);

        // Move to initial position
        moveToInitialPosition();
    }

private:
    void moveToInitialPosition()
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x_position_;
        target_pose.position.y = y_position_;
        target_pose.position.z = z_position_;

        tf2::Quaternion quaternion;
        quaternion.setRPY(rx_, ry_, rz_);
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

    void handleMoveRequest(const std::shared_ptr<custom_msgs::srv::MoveToPoint::Request> request,
                           std::shared_ptr<custom_msgs::srv::MoveToPoint::Response> response)
    {
        double new_x = request->point.x;
        double new_y = request->point.y;
        double new_z = request->point.z;
        // 数据为空时，回到初始位置
        if (new_x == 0 && new_y == 0 && new_z == 0)
        {
            moveToInitialPosition();
            response->success = true;
            response->message = "Moved to initial position.";
            return;
        }
        RCLCPP_INFO(this->get_logger(), "%f,%f,%f", new_x, new_y, new_z);
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = new_x;
        target_pose.position.y = new_y;
        target_pose.position.z = new_z;

        tf2::Quaternion quaternion;
        quaternion.setRPY(rx_, ry_, rz_);
        target_pose.orientation.x = quaternion.x();
        target_pose.orientation.y = quaternion.y();
        target_pose.orientation.z = quaternion.z();
        target_pose.orientation.w = quaternion.w();

        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "Move request planning success: %s", success ? "True" : "False");

        if (success)
        {
            move_group_->move();
            response->success = true;
            response->message = "Moved to new position.";
        }
        else
        {
            response->success = false;
            response->message = "Failed to move to new position.";
        }
    }

    string model_;
    double x_position_, y_position_, z_position_;
    double rx_, ry_, rz_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Service<custom_msgs::srv::MoveToPoint>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);

    auto node = std::make_shared<JakaPlanner>();
    node->initializeMoveGroup(); // 初始化 MoveGroupInterface

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}