#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_example/utils.hpp"

void SetupScene()
{
  const auto [objects, colors] = []
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects(3);

    // Place first table
    objects[0].id = "table1";
    objects[0].header.frame_id = "world";
    objects[0].primitives.resize(1);
    objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[0].primitives[0].dimensions.resize(3);
    objects[0].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.4;
    objects[0].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.4;
    objects[0].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.4;
    objects[0].primitive_poses.resize(1);
    objects[0].primitive_poses[0].position.x = 0.6;
    objects[0].primitive_poses[0].position.y = -0.25;
    objects[0].primitive_poses[0].position.z = 0.2;
    objects[0].primitive_poses[0].orientation.w = 1.0;
    objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Place second table
    objects[1].id = "table2";
    objects[1].header.frame_id = "world";
    objects[1].primitives.resize(1);
    objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[1].primitives[0].dimensions.resize(3);
    objects[1].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.4;
    objects[1].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.4;
    objects[1].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.4;
    objects[1].primitive_poses.resize(1);
    objects[1].primitive_poses[0].position.x = 0.6;
    objects[1].primitive_poses[0].position.y = 0.25;
    objects[1].primitive_poses[0].position.z = 0.2;
    objects[1].primitive_poses[0].orientation.w = 1.0;
    objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Place cylinder on the first table
    objects[2].id = "cylinder";
    objects[2].header.frame_id = "world";
    objects[2].primitives.resize(1);
    objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    objects[2].primitives[0].dimensions.resize(2);
    objects[2].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.1;
    objects[2].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.02;
    objects[2].primitive_poses.resize(1);
    objects[2].primitive_poses[0].position.x = 0.6;
    objects[2].primitive_poses[0].position.y = 0.25;
    objects[2].primitive_poses[0].position.z = 0.45 + 1e-3;
    objects[2].primitive_poses[0].orientation.w = 1.0;
    objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Specify the color of each object in the scene
    std::vector<moveit_msgs::msg::ObjectColor> colors(3);

    // Specify the first table's color
    colors[0].color.r = 1.0;
    colors[0].color.g = 0.0;
    colors[0].color.b = 0.0;
    colors[0].color.a = 1.0;

    // Specify the second table's color
    colors[1].color.r = 0.0;
    colors[1].color.g = 0.0;
    colors[1].color.b = 1.0;
    colors[1].color.a = 1.0;

    // Specify the cone's color
    colors[2].color.r = 0.5;
    colors[2].color.g = 0.0;
    colors[2].color.b = 0.5;
    colors[2].color.a = 1.0;

    return std::make_pair(objects, colors);
  }();

  // Add the collision objects to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(objects, colors);
}

static const geometry_msgs::msg::Point GetObjectPosition(const std::string object_id)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  return planning_scene_interface.getObjects({object_id})[object_id].pose.position;
}

void SecondScenario(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
  moveit_visual_tools.prompt("Press 'Next' to start the second scenario");

  SetupScene();

  // TODO 1
  // Move the end-effector above the cylinder and make sure that it is facing down
  //
  // You have to:
  // - Find the position of the cylinder
  // - Create the pick_up_pose
  // - Move the robot to the specified pose
  // ...

  // TODO 2
  // Simulate picking up the cylinder by attaching it to the end-effector
  // ...

  // TODO 3
  // Set up orientation constraint to keep end-effector pointing down during movement
  //
  // You have to:
  // - Describe the constraint with some tolerance
  // - Apply this constraint to the move_group_interface
  // ...

  // TODO 4
  // Move the end-effector above the other table (i.e. table1)
  //
  // You have to:
  // - Find the position of table1
  // - Create the drop_off_pose
  // - Move the robot to the specified pose
  // ...

  // TODO 5
  // Drop the cylinder onto the table by detaching it from the robot
  // ...

  // Clear path constraints
  move_group_interface.clearPathConstraints();

  // Move to the home position
  MoveToHome(move_group_interface, moveit_visual_tools, logger);
}
