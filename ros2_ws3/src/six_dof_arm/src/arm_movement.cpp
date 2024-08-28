#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class ArmMovementNode : public rclcpp::Node {
public:
  ArmMovementNode() : Node("arm_movement") {
    RCLCPP_INFO(this->get_logger(), "ArmMovement node has started successfully.");

    // Initialize MoveIt components
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface(this->shared_from_this(), "arm");

    // Set up the target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 1.0;
    target_pose.position.y = 0.0;
    target_pose.position.z = 1.0;
    target_pose.orientation.w = 1.0;

    move_group_interface.setPoseTarget(target_pose);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
    }
  }
private:
  // Any private members or methods can be declared here
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmMovementNode>());
  rclcpp::shutdown();
  return 0;
}
