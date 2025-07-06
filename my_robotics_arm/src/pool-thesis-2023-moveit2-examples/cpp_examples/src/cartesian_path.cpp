#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("gripper_open_with_obstacle");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("gripper_open_with_obstacle");

  // Create the MoveIt Move Group Interface for panda hand (gripper)
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Create the Planning Scene Interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";  // Give the box a unique ID

    shape_msgs::msg::SolidPrimitive primitive;
    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.2;
    primitive.dimensions[primitive.BOX_Y] = 0.2;
    primitive.dimensions[primitive.BOX_Z] = 0.2;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // Quaternion initialization, no rotation
    box_pose.position.x = 0.4;  // X position of the box
    box_pose.position.y = 0.4;  // Y position of the box
    box_pose.position.z = 1.0; // Z position of the box

    // Add the primitive and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;  // Add this object to the scene

    return collision_object;
  }();

  // Add the collision object to the planning scene
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.applyCollisionObjects(collision_objects);

  // Named goal for gripper to open (you can also use "close" to close the gripper)
  move_group_interface.setNamedTarget("ready");

  // Create a plan to that target pose and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // If the plan is successful, execute the plan
  if(success) {
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown
  rclcpp::shutdown();
  return 0;
}
