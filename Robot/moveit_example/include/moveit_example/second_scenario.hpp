#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <regex>

#include "moveit_example/utils.hpp"

//király bábu létrehozás
moveit_msgs::msg::CollisionObject MakeKiraly(const std::string &id, double x, double y, double z)
{
    moveit_msgs::msg::CollisionObject kiraly;
    kiraly.id = id;
    kiraly.header.frame_id = "world";

    kiraly.primitives.resize(1);
    kiraly.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    kiraly.primitives[0].dimensions = {0.1, 0.015};  // height, radius

    kiraly.primitive_poses.resize(1);
    kiraly.primitive_poses[0].position.x = x;
    kiraly.primitive_poses[0].position.y = y;
    //mező közepe + fél mező magasság + fél bábu magasság + apróóó eltolás, hogy ne ütközzön
    kiraly.primitive_poses[0].position.z = z + 0.01 +  0.05 + 1e-3; 
    kiraly.primitive_poses[0].orientation.w = 1.0;

    kiraly.operation = moveit_msgs::msg::CollisionObject::ADD;
    return kiraly;
}

//paraszt bábu létrehozás
moveit_msgs::msg::CollisionObject MakeParaszt(const std::string &id, double x, double y, double z)
{
    moveit_msgs::msg::CollisionObject paraszt;
    paraszt.id = id;
    paraszt.header.frame_id = "world";

    paraszt.primitives.resize(1);
    paraszt.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    paraszt.primitives[0].dimensions = {0.05, 0.01};  // height, radius

    paraszt.primitive_poses.resize(1);
    paraszt.primitive_poses[0].position.x = x;
    paraszt.primitive_poses[0].position.y = y;
    //mező közepe + fél mező magasság + fél bábu magasság + apróóó eltolás, hogy ne ütközzön
    paraszt.primitive_poses[0].position.z = z + 0.01 + 0.025 + 1e-3;
    paraszt.primitive_poses[0].orientation.w = 1.0;

    paraszt.operation = moveit_msgs::msg::CollisionObject::ADD;
    return paraszt;
}

//bábu nevének ellenőrzése
bool is_valid_square(const std::string& s)
{
    static const std::regex pattern("^square_([1-9]|[1-5][0-9]|6[0-4])$");
    return std::regex_match(s, pattern);
}

void SetupScene()
{
  const auto [objects, colors] = []
    {
        constexpr int N = 8;
        constexpr double cell_size = 0.04;   // 4 cm
        constexpr double height = 0.02;      // 2 cm

        //tábla közepének pozíciója
        double center_x = 0.3;
        double center_y = 0.0;
        double center_z = 0.0;

        // Fél tábla mérete a középre igazításhoz
        constexpr double half_board = (N * cell_size) / 2.0;

        // CollisionObject lista
        std::vector<moveit_msgs::msg::CollisionObject> objects(N * N);

        int id = 0;
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {

                objects[id].id = "square_" + std::to_string(id+1);
                objects[id].header.frame_id = "world";

                // BOX shape
                objects[id].primitives.resize(1);
                objects[id].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
                objects[id].primitives[0].dimensions = {
                    cell_size,
                    cell_size,
                    height
                };

                // Mező pozíciója a tábla középpontjához viszonyítva
                objects[id].primitive_poses.resize(1);
                objects[id].primitive_poses[0].position.x =
                    center_x + (j * cell_size - half_board + cell_size / 2.0);

                objects[id].primitive_poses[0].position.y =
                    center_y + (i * cell_size - half_board + cell_size / 2.0);

                // tábla síkja center_z, a mezők közepe a felette lévő 0.01 m
                objects[id].primitive_poses[0].position.z = center_z + height / 2.0;

                objects[id].primitive_poses[0].orientation.w = 1.0;
                objects[id].operation = moveit_msgs::msg::CollisionObject::ADD;

                id++;
            }
        }

        // tábla mezőinek színe
        std::vector<moveit_msgs::msg::ObjectColor> colors(N * N);

        id = 0;
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {

                bool white = ((i + j) % 2 == 0);

                if (white)
                {
                    colors[id].color.r = 1.0;
                    colors[id].color.g = 1.0;
                    colors[id].color.b = 1.0;
                    colors[id].color.a = 1.0;
                }else{
                    colors[id].color.r = 0.0;
                    colors[id].color.g = 0.0;
                    colors[id].color.b = 0.0;
                    colors[id].color.a = 1.0;
                }

                colors[id].id = "square_" + std::to_string(id+1);

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

void Move_Piece(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
    moveit_visual_tools.prompt("Kattnints Next-re, hogy betöltsd a sakktáblát");

    //sakktábla betöltése
    SetupScene();

    moveit_visual_tools.prompt("Sakktábla betöltve, kattints Next-re, hogy kiválaszd a sakkfigura típusát");

    //bekérés a felhasználótól
    std::string piece_type;
    std::cout << "Válaszd ki a sakkfigura típusát(K|P):";
    std::cin >> piece_type;

    if(piece_type == "K"){
        RCLCPP_INFO(logger, "Sakkfigura tipusa: király");
    }else{
        RCLCPP_INFO(logger, "Sakkfigura tipusa: paraszt");
    }
    
    //mező kiválasztása, ahova a sakkfigura kerül
    moveit_visual_tools.prompt("Kattints Next-re, hogy kiválaszdd a sakkfigura mezőjét a sakktáblán");

    std::string target_square;
    std::cout << "Válaszd ki, hogy hova kerüljön a sakkfigura(pl. square_4): ";
    std::cin >> target_square;

    //bemenet helyességének ellenőrzése
    while(!is_valid_square(target_square)){
        std::cout << "Helytelen mező, válaszd ki újra(pl. square_4): ";
        std::cin >> target_square;
    }

    RCLCPP_INFO(logger, "Sakkfigura pozíciója: %s", target_square.c_str());

    // pozíció lekérése
    geometry_msgs::msg::Point pos = GetObjectPosition(target_square);

    // bábu létrehozása és megjelenítése
    moveit_msgs::msg::CollisionObject babu;
    if(piece_type == "K"){
        babu = MakeKiraly("babu", pos.x, pos.y, pos.z);
    }else{
        babu = MakeParaszt("babu", pos.x, pos.y, pos.z);
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(babu);

    RCLCPP_INFO(logger, "Bábu lehelyezve a %s mezőre", target_square.c_str());

    moveit_visual_tools.prompt("Bábu lehelyezve. Kattints Next-re, hogy a robot felvegye a bábut");

    //constraintek beállítása
    const auto constraints = []
    {
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = "tool0";
        ocm.header.frame_id = "world";

        ocm.orientation.x = 0.0;
        ocm.orientation.y = std::sin(Constants::PI / 2);
        ocm.orientation.z = 0.0;
        ocm.orientation.w = std::cos(Constants::PI / 2);

        // Specify max deviation in radians
        ocm.absolute_x_axis_tolerance = 0.5;
        ocm.absolute_y_axis_tolerance = 0.5;
        ocm.absolute_z_axis_tolerance = 0.5;
        ocm.weight = 1.0;

        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);
        return constraints;
    }();
    move_group_interface.setPathConstraints(constraints);

    //bábu felvétele
    const auto babu_pos = GetObjectPosition("babu");
    RCLCPP_INFO(logger, "Bábu pozíció: %f %f %f", babu_pos.x, babu_pos.y, babu_pos.z);
    const auto pick_up_pose = [&babu_pos, &piece_type]
    {
        geometry_msgs::msg::Pose msg;

        msg.orientation.x = 0.0;
        msg.orientation.y = std::sin(Constants::PI / 2);
        msg.orientation.z = 0.0;
        msg.orientation.w = std::cos(Constants::PI / 2);

        msg.position.x = babu_pos.x;
        msg.position.y = babu_pos.y;
        if(piece_type == "K"){
            //bábu közepe + bábu felének magassága + 0.01
            msg.position.z = babu_pos.z + 0.05 + 0.01;
        }else{
            msg.position.z = babu_pos.z + 0.025 + 0.01;
        }
        return msg;
    }();

    MoveToPose(pick_up_pose, move_group_interface, moveit_visual_tools, logger);

    move_group_interface.attachObject("babu");
    moveit_visual_tools.prompt("Babu felvéve, nyomd meg a Next-et, hogy kiválaszd a célmezőt");

    //célmező kiválasztása
    std::string destination_square;
    std::cout << "Válaszd ki, hogy hova mozgassuk a sakkfigurát(pl. square_4): ";
    std::cin >> destination_square;

    //bemenet formátumának leellenőrzése
    while(!is_valid_square(destination_square)){
        std::cout << "Helytelen mező, válaszd ki újra(pl. square_4): ";
        std::cin >> destination_square;
    }

    RCLCPP_INFO(logger, "Desztináció: %s", target_square.c_str());

    //mozgatás a célmezőre
    const auto drop_off_position = GetObjectPosition(destination_square);
    RCLCPP_INFO(logger, "Target mező: %f %f %f", drop_off_position.x, drop_off_position.y, drop_off_position.z);

    const auto drop_off_pose = [&drop_off_position, &piece_type]
    {
        geometry_msgs::msg::Pose msg;

        msg.orientation.x = 0.0;
        msg.orientation.y = std::sin(Constants::PI / 2);
        msg.orientation.z = 0.0;
        msg.orientation.w = std::cos(Constants::PI / 2);

        msg.position.x = drop_off_position.x;
        msg.position.y = drop_off_position.y;
        if(piece_type == "K"){
            //cémező közepe + célmező felének magassága + bábu magassága + 0.01 + apró eltolás
            msg.position.z = drop_off_position.z + 0.01 + 0.10 + 0.01 + 1e-3;
        }else{
            msg.position.z = drop_off_position.z + 0.01 + 0.05 + 0.01 + 1e-3;
        }
        return msg;
    }();

    move_group_interface.setPlanningTime(15.0);
    MoveToPose(drop_off_pose, move_group_interface, moveit_visual_tools, logger);

    move_group_interface.detachObject("babu");
    moveit_visual_tools.prompt("Bábu elhelyezve a kijelölt helyre, nyomj Next-et a folytatáshoz");

    move_group_interface.clearPathConstraints();

    MoveToHome(move_group_interface, moveit_visual_tools, logger);
}



