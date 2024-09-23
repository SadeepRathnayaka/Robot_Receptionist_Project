#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1;

class MovePoseNode : public rclcpp::Node
{
public:
  MovePoseNode() : Node("move_pose_node"),
                   move_group_interface(std::make_shared<rclcpp::Node>("moveit_node"), "arm")
  {
    sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "pose_topic", 5, std::bind(&MovePoseNode::msgCallback, this, _1));

    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setPoseReferenceFrame("base_link");
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(3.14159);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  geometry_msgs::msg::Pose target_pose;
  
  void msgCallback(const geometry_msgs::msg::Pose &msg) 
  {
    if (msg.position.x != target_pose.position.x || msg.position.y != target_pose.position.y || msg.position.z != target_pose.position.z )
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "New pose received");

        target_pose.position.x = msg.position.x;
        target_pose.position.y = msg.position.y;
        target_pose.position.z = msg.position.z;
        target_pose.orientation.x = msg.orientation.x;
        target_pose.orientation.y = msg.orientation.y;
        target_pose.orientation.z = msg.orientation.z;

        move_group_interface.setStartStateToCurrentState();
        move_group_interface.setPoseTarget(target_pose);

        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface = move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
        }();

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planner SUCCEEDED, moving the arm and the gripper");
            move_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "One or more planners failed!");
            return;
        }
    }
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovePoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
}