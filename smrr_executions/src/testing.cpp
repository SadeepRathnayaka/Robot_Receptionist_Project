// void move_robot(const std::shared_ptr<rclcpp::Node> node)
// {
//     auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "arm");

//     geometry_msgs::msg::Pose target_pose;

//     double x = 0.5;

//     for (int i= 0 ; i < 2; i++)
//     {
//     target_pose.position.x = x;
//     target_pose.position.y = -0.2;  
//     target_pose.position.z = 1.2;
//     target_pose.orientation.x = 0.0;
//     target_pose.orientation.y = 0.0;
//     target_pose.orientation.z = 0.0;
//     target_pose.orientation.w = 0.1;

//     move_group_interface.setStartStateToCurrentState();
//     move_group_interface.setPlanningTime(5.0);
//     move_group_interface.setPoseReferenceFrame("base_link");
//     move_group_interface.setPoseTarget(target_pose);
//     move_group_interface.setGoalPositionTolerance(0.001);
//     move_group_interface.setGoalOrientationTolerance(3.14159);

//     // Create a plan to that target pose
//     auto const [success, plan] = [&move_group_interface]{
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//     }();


//     if(success)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//                     "Planner SUCCEED, moving the arm and the gripper");
//         move_group_interface.execute(plan);
//     }
//     else
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
//                      "One or more planners failed!");
//         return;
//     }

//     x += 0.05 ;

//     }

//     // std::vector<double> arm_joint_goal {1.5, 0.0, 0.5, 0.1};

//     // bool arm_within_bounds = move_group_interface.setJointValueTarget(arm_joint_goal);

//     // if (!arm_within_bounds)
//     // {
//     //     RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
//     //                 "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
//     //     return;
//     // }

//     // moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
//     // bool arm_plan_success = (move_group_interface.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     // if(arm_plan_success)
//     // {
//     //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//     //                 "Planner SUCCEED, moving the arme and the gripper");
//     //     move_group_interface.execute(arm_plan);
//     // }
//     // else
//     // {
//     //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
//     //                  "One or more planners failed!");
//     //     return;
//     // }
// }