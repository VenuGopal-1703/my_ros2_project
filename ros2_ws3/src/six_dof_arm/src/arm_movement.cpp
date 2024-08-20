#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/robot_model_loader.h>

class ArmMovement : public rclcpp::Node
{
public:
    ArmMovement() : Node("arm_movement")
    {
        RCLCPP_INFO(this->get_logger(), "ArmMovement node started successfully");

        // Initialize MoveIt! interface
        moveit::planning_interface::PlanningInterfacePtr move_group_interface =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");

        // Define target pose
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = 0.4;
        target_pose.pose.position.y = 0.0;
        target_pose.pose.position.z = 0.4;
        target_pose.pose.orientation.w = 1.0;

        // Plan to the target pose
        move_group_interface->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_interface->plan(plan);
        move_group_interface->execute(plan);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmMovement>());
    rclcpp::shutdown();
    return 0;
}
