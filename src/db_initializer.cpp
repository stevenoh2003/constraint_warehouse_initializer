#include "db_initializer.h"
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <boost/math/constants/constants.hpp>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/ros.h>
#include <XmlRpcValue.h> 

DBInitializer::DBInitializer(const std::string &robot_description, int argc, char **argv)
    : _robot_description(robot_description)
{
    ros::init(argc, argv, "initialize_demo_db", ros::init_options::AnonymousName);
    _psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description);
}

bool DBInitializer::initialize()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string host;
    int port;
    ros::param::param<std::string>("~warehouse_host", host, "localhost");
    ros::param::param<int>("~warehouse_port", port, 33829);

    if (!_psm->getPlanningScene())
    {
        ROS_ERROR("Unable to initialize PlanningSceneMonitor");
        return false;
    }

    _conn = moveit_warehouse::loadDatabase();
    _conn->setParams(host, port);
    if (!_conn->connect())
    {
        ROS_ERROR("Failed to connect to the database");
        return false;
    }

    loadConstraintsFromROSParam();

    ROS_INFO("Default MoveIt Warehouse was initialized successfully.");
    return true;
}

void DBInitializer::addConstraint(const std::string &group_name, const moveit_msgs::Constraints &constraint, const std::string &constraint_name, const bool &reset)
{
    moveit_warehouse::ConstraintsStorage cs(_conn);

    if (reset)
    {
        cs.reset();
    }

    cs.addConstraints(constraint, _psm->getRobotModel()->getName(), group_name);
    ROS_INFO("Added constraint: '%s'", constraint_name.c_str());
}

planning_scene_monitor::PlanningSceneMonitorPtr DBInitializer::getPlanningSceneMonitorPtr() const
{
    return _psm;
}

void DBInitializer::loadConstraintsFromROSParam()
{
    bool reset = false; // Default value
    if (!ros::param::get("/reset_constraints", reset))
    {
        ROS_WARN("Parameter /reset_constraints not found. Using default: false");
    }

    XmlRpc::XmlRpcValue constraint_list;
    if (ros::param::get("/constraints", constraint_list))
    {
        ROS_ASSERT(constraint_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int i = 0; i < constraint_list.size(); ++i)
        {
            ROS_ASSERT(constraint_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
            auto &entry = constraint_list[i];

            if (entry["type"] == "orientation")
            {
                moveit_msgs::OrientationConstraint ocm;
                ocm.link_name = static_cast<std::string>(entry["link_name"]);
                ocm.header.frame_id = _psm->getRobotModel()->getModelFrame();
                ocm.orientation.x = static_cast<double>(entry["orientation_x"]);
                ocm.orientation.y = static_cast<double>(entry["orientation_y"]);
                ocm.orientation.z = static_cast<double>(entry["orientation_z"]);
                ocm.orientation.w = static_cast<double>(entry["orientation_w"]);
                ocm.absolute_x_axis_tolerance = static_cast<double>(entry["absolute_x_axis_tolerance"]);
                ocm.absolute_y_axis_tolerance = static_cast<double>(entry["absolute_y_axis_tolerance"]);
                ocm.absolute_z_axis_tolerance = static_cast<double>(entry["absolute_z_axis_tolerance"]);
                ocm.weight = static_cast<double>(entry["weight"]);

                moveit_msgs::Constraints cmsg;
                cmsg.orientation_constraints.push_back(ocm);
                cmsg.name = ocm.link_name + ":" + static_cast<std::string>(entry["constraint_name"]);

                // Use the reset value read from ROS parameters
                addConstraint(static_cast<std::string>(entry["move_group"]), cmsg, cmsg.name, reset);
            }
        }
    }
    else
    {
        ROS_ERROR("Failed to load constraints from ROS parameters.");
    }
}
