#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "std_msgs/msg/string.hpp"

#include "Motionplanning_arms.hpp"
#include "RobotTaskStatus.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <play_motion2_msgs/action/play_motion2.hpp>
#include <control_msgs/action/point_head.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <chrono>
#include <vector>

#include <std_msgs/msg/int32.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);


  // Create a shared pointer to your node
  auto node = std::make_shared<icr_Motionplanning_arms>();

  // home gripper
  node->GripperControl("OPEN");


  // detect a marker position

  // create the table obstacle
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
  planning_scene_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>(
    "/planning_scene", rclcpp::QoS(1));

  // Create a publisher for the current layer
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_layer_publisher_;
  current_layer_publisher_ = node->create_publisher<std_msgs::msg::Int32>(
    "/current_layer", rclcpp::QoS(10));

  


// // Add obj table in pose 0.4 0.0 0.5
//   geometry_msgs::msg::PoseStamped obstacle_pose;
//   obstacle_pose.header.frame_id = "base_footprint";
//   obstacle_pose.pose.position.x = 0.4;
//   obstacle_pose.pose.position.y = 0.0;
//   obstacle_pose.pose.position.z = 0.82;
//   obstacle_pose.pose.orientation.x = 0.0;
//   obstacle_pose.pose.orientation.y = 0.0;
//   obstacle_pose.pose.orientation.z = 0.0;
//   obstacle_pose.pose.orientation.w = 1.0;

  // move to the grasp pose
  // grasp the object
  // move the arm to the correct pose 
  // open the gripper

  // move the grasp pose

  int current_layer = 0;
  bool first_brick = false;

  // bool first_it = true;


  double brick_length = 0.07 - 0.025;
  double brick_height = 0.015;

  geometry_msgs::msg::PoseStamped obstacle_pose;
  obstacle_pose.header.frame_id = "base_footprint";
  obstacle_pose.pose.position.x = 0.55;
  obstacle_pose.pose.position.y = 0.0;
  obstacle_pose.pose.position.z = 0.44;
  obstacle_pose.pose.orientation.x = 0.0;
  obstacle_pose.pose.orientation.y = 0.0;
  obstacle_pose.pose.orientation.z = 0.0;
  obstacle_pose.pose.orientation.w = 1.0;
  moveit_msgs::msg::PlanningScene planning_scene_msg =
    node->Add_Obstacle(obstacle_pose, "Table");
  planning_scene_publisher_->publish(planning_scene_msg);
  RCLCPP_INFO(node->get_logger(), "Obstacle 'Table' added to the planning scene.");

  while (rclcpp::ok()) {


    // Publish the current layer value
    std_msgs::msg::Int32 current_layer_msg;
    current_layer_msg.data = current_layer;
    current_layer_publisher_->publish(current_layer_msg);

    if(!first_brick) {
      current_layer++;
    }

    first_brick = !first_brick;


    

    geometry_msgs::msg::Pose grasp_pose;
    grasp_pose.position.x = 0.5;
    grasp_pose.position.y = 0.0;
    grasp_pose.position.z = 1.1;

    // Simple orientation (quaternion), facing forward
    grasp_pose.orientation.x = 0.0;
    grasp_pose.orientation.y = 0.7071;
    grasp_pose.orientation.z = 0.0;
    grasp_pose.orientation.w = 0.7071;


    bool motion_planning_success = false;
    while (!motion_planning_success) {
      try {
      node->motion_planning_control(grasp_pose, RobotTaskStatus::Arm::ARM_torso);  // or "arm" depending on your group name
      RCLCPP_INFO(node->get_logger(), "grasp_pose: Motion planning succeeded.");
      motion_planning_success = true;
      } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s. Retrying...", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Wait before retrying
      }
    }

    node->GripperControl("CLOSE");


    if(!first_brick) {
        // geometry_msgs::msg::PoseStamped obstacle_pose;
        obstacle_pose.header.frame_id = "base_footprint";
        obstacle_pose.pose.position.x = 0.55;
        obstacle_pose.pose.position.y = 0.0;
        obstacle_pose.pose.position.z = 0.44 + brick_height * (double(current_layer));
        obstacle_pose.pose.orientation.x = 0.0;
        obstacle_pose.pose.orientation.y = 0.0;
        obstacle_pose.pose.orientation.z = 0.0;
        obstacle_pose.pose.orientation.w = 1.0;
        moveit_msgs::msg::PlanningScene planning_scene_msg =
          node->Add_Obstacle(obstacle_pose, "Table");
        planning_scene_publisher_->publish(planning_scene_msg);
        RCLCPP_INFO(node->get_logger(), "Obstacle 'Table' added to the planning scene.");
    }

    // if(first_it){
    //   moveit_msgs::msg::PlanningScene planning_scene_msg =
    //     node->Add_Obstacle(obstacle_pose, "Table");
    //   planning_scene_publisher_->publish(planning_scene_msg);
    //   first_it = false;
    //   RCLCPP_INFO(node->get_logger(), "Obstacle 'Table' added to the planning scene.");
    // }


    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    geometry_msgs::msg::Pose brick_target_pose;
    // brick_target_pose = obstacle_pose.pose;  // Use the same pose as the obstacle



    double brick_x_offset;
    double brick_y_offset;
    double brick_z_offset;

    if(first_brick && current_layer % 2 != 0) {
      brick_x_offset = brick_length/2.0;  // First brick in the first layer
      brick_y_offset = 0.0;  // No offset in y for the first brick
    } else if(!first_brick && current_layer % 2 != 0) {
      brick_x_offset = -brick_length/2.0;  // First brick in subsequent layers
      brick_y_offset = 0.0;  // No offset in y for the first brick
    } else if(first_brick && current_layer % 2 == 0) {
      brick_x_offset = 0.0;  // Second brick in the first layer
      brick_y_offset = brick_length/2.0;  // No offset in y for the second brick
    } else {
      brick_x_offset = 0.0;  // Second brick in subsequent layers
      brick_y_offset = -brick_length/2.0;  // No offset in y for the second brick
    }

    // double table_offset = 0.03;
    double table_offset = 0.0015;
    double ee_offset = 0.2333;  // Adjust this value based on your end effector's height
    // brick_z_offset = brick_height * double(current_layer) + table_offset + ee_offset;  // Adjust height for each layer
    brick_z_offset = table_offset + ee_offset;  // Adjust height for each layer


    brick_target_pose.position.x = obstacle_pose.pose.position.x + brick_x_offset;
    brick_target_pose.position.y = obstacle_pose.pose.position.y + brick_y_offset;
    brick_target_pose.position.z = obstacle_pose.pose.position.z + brick_z_offset;


    

    if(current_layer % 2 == 0){
        brick_target_pose.orientation.x = -0.5;
        brick_target_pose.orientation.y = 0.5;
        brick_target_pose.orientation.z = 0.5;
        brick_target_pose.orientation.w = 0.5;
    }
    else {
        // Simple orientation (quaternion), facing forward
        brick_target_pose.orientation.x = grasp_pose.orientation.x;
        brick_target_pose.orientation.y = grasp_pose.orientation.y;
        brick_target_pose.orientation.z = grasp_pose.orientation.z;
        brick_target_pose.orientation.w = grasp_pose.orientation.w;
    }

    // brick_target_pose.position.x = 0.4;
    // brick_target_pose.position.y = 0.0;
    // brick_target_pose.position.z = 0.9;
    // // brick_target_pose.pose.position.z += 0.1;  // Adjust height for the brick

    // // Simple orientation (quaternion), facing forward
    // brick_target_pose.orientation.x = 0.0;
    // brick_target_pose.orientation.y = 0.7071;
    // brick_target_pose.orientation.z = 0.0;
    // brick_target_pose.orientation.w = 0.7071;

    motion_planning_success = false;
    while (!motion_planning_success) {
      try {
        node->motion_planning_control(brick_target_pose, RobotTaskStatus::Arm::ARM_torso);  // or "arm" depending on your group name
        RCLCPP_INFO(node->get_logger(), "brick_target_pose: Motion planning succeeded.");
        motion_planning_success = true;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Wait before retrying
      }
      RCLCPP_ERROR(node->get_logger(), "Brick Z Offset: %f", brick_z_offset);
      RCLCPP_ERROR(node->get_logger(), "brick_target_pose.position.z: %f", brick_target_pose.position.z);
    }

    node->GripperControl("OPEN");


  }


  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  /////////////////////////////////////////////////////////////////

  // grasp_pose.position.x = 0.4;
  // grasp_pose.position.y = 0.0;
  // grasp_pose.position.z = 0.8;

  // // Simple orientation (quaternion), facing forward
  // grasp_pose.orientation.x = 0.0;
  // grasp_pose.orientation.y = 0.7071;
  // grasp_pose.orientation.z = 0.0;
  // grasp_pose.orientation.w = 0.7071;



  // // You can spin in a separate thread or just call your function directly
  // try {
  //   double lift_value = 0.0;   // Example lift value within limits
  //   // node->TorsoControl(lift_value);
  // } catch (const std::exception & e) {
  //   RCLCPP_ERROR(node->get_logger(), "Exception in TorsoControl: %s ", e.what());
  // }

  // try {
  //   node->motion_planning_control(grasp_pose, RobotTaskStatus::Arm::ARM_torso);  // or "arm" depending on your group name
  //   RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
  // } catch (const std::exception & e) {
  //   RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
  // }


  // node->GripperControl("CLOSE");

  rclcpp::shutdown();
  return 0;
}
