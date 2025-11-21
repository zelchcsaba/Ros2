#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_example/utils.hpp"

moveit_msgs::msg::CollisionObject MakePawn(const std::string &id, double x, double y, double z)
{
    moveit_msgs::msg::CollisionObject pawn;
    pawn.id = id;
    pawn.header.frame_id = "world";

    pawn.primitives.resize(1);
    pawn.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    pawn.primitives[0].dimensions = {0.05, 0.015};  // height, radius

    pawn.primitive_poses.resize(1);
    pawn.primitive_poses[0].position.x = x;
    pawn.primitive_poses[0].position.y = y;
    pawn.primitive_poses[0].position.z = z + 0.025; // ráül a mezőre
    pawn.primitive_poses[0].orientation.w = 1.0;

    pawn.operation = moveit_msgs::msg::CollisionObject::ADD;
    return pawn;
}

void SetupScene()
{
  const auto [objects, colors] = []
{
    constexpr int N = 8;
    constexpr double cell_size = 0.04;   // 4 cm
    constexpr double height = 0.02;      // 2 cm

    // -----> A TÁBLA KÖZEPÉNEK POZÍCIÓJA (EZT SZABADON ÁTÍRHATOD) <-----
    double center_x = 0.3;
    double center_y = 0.0;
    double center_z = 0.0;
    // -----------------------------------------------------------------

    // Fél tábla mérete a középre igazításhoz
    constexpr double half_board = (N * cell_size) / 2.0;

    // CollisionObject lista
    std::vector<moveit_msgs::msg::CollisionObject> objects(N * N);

    int id = 0;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            auto& obj = objects[id];

            obj.id = "square_" + std::to_string(id);
            obj.header.frame_id = "world";

            // BOX shape
            obj.primitives.resize(1);
            obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
            obj.primitives[0].dimensions = {
                cell_size,
                cell_size,
                height
            };

            // Mező pozíciója a tábla középpontjához viszonyítva
            obj.primitive_poses.resize(1);
            obj.primitive_poses[0].position.x =
                center_x + (j * cell_size - half_board + cell_size / 2.0);

            obj.primitive_poses[0].position.y =
                center_y + (i * cell_size - half_board + cell_size / 2.0);

            // tábla síkja center_z, a mezők közepe a felette lévő 0.01 m
            obj.primitive_poses[0].position.z =
                center_z + height / 2.0;

            obj.primitive_poses[0].orientation.w = 1.0;
            obj.operation = moveit_msgs::msg::CollisionObject::ADD;

            id++;
        }
    }

    // COLORS
    std::vector<moveit_msgs::msg::ObjectColor> colors(N * N);

    id = 0;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            auto& col = colors[id];

            bool white = ((i + j) % 2 == 0);

            if (white)
            {
                col.color.r = 1.0;
                col.color.g = 1.0;
                col.color.b = 1.0;
                col.color.a = 1.0;
            }
            else
            {
                col.color.r = 0.0;
                col.color.g = 0.0;
                col.color.b = 0.0;
                col.color.a = 1.0;
            }

            col.id = "square_" + std::to_string(id);

            id++;
        }
    }

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

   moveit_visual_tools.prompt("Chessboard loaded. Press 'Next' to select a square.");

  // 2) bekérés a felhasználótól
  std::string target_square;
  std::cout << "Enter square name (e.g. square_12): ";
  std::cin >> target_square;

  RCLCPP_INFO(logger, "User selected: %s", target_square.c_str());

  // 3) pozíció lekérése
  geometry_msgs::msg::Point pos = GetObjectPosition(target_square);

  // 4) bábu létrehozása és megjelenítése
  moveit_msgs::msg::CollisionObject pawn =
      MakePawn("pawn_white", pos.x, pos.y, pos.z);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(pawn);

  RCLCPP_INFO(logger, "Pawn placed on %s", target_square.c_str());

  moveit_visual_tools.prompt("Pawn added. Press 'Next' to move the robot to this square.");

  const auto pawn_pos = GetObjectPosition("pawn_white");

  geometry_msgs::msg::Pose pick_up_pose;

  pick_up_pose.orientation.x = 0.0;
  pick_up_pose.orientation.y = std::sin(Constants::PI / 2);
  pick_up_pose.orientation.z = 0.0;
  pick_up_pose.orientation.w = std::cos(Constants::PI / 2);

  pick_up_pose.position.x = pawn_pos.x;
  pick_up_pose.position.y = pawn_pos.y;
  pick_up_pose.position.z = pawn_pos.z + 0.06;  // nagyobb magasság

  MoveToPose(pick_up_pose, move_group_interface, moveit_visual_tools, logger);

  moveit_visual_tools.prompt("Done. Press 'Next' to return to home pose.");

  MoveToHome(move_group_interface, moveit_visual_tools, logger);
}



