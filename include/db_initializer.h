#ifndef DB_INITIALIZER_H
#define DB_INITIALIZER_H

#include <ros/ros.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <string>
#include <moveit/kinematic_constraints/utils.h>
// Include the necessary header for database connection from MoveIt warehouse
#include <warehouse_ros/database_connection.h>

class DBInitializer
{
public:
    // Constructor: Initialize the DBInitializer with robot description, argc, and argv for ROS initialization
    DBInitializer(const std::string &robot_description, int argc, char **argv);

    // Initialize the DBInitializer object, setting up necessary components and connections
    bool initialize();

    // Add a constraint to the warehouse database with an option to reset all constraints for a given group
    void addConstraint(const std::string &group_name,
                       const moveit_msgs::Constraints &constraint,
                       const std::string &constraint_name, const bool &reset_all_constraints);

    // Retrieve the PlanningSceneMonitor pointer for accessing planning scene related operations
    planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitorPtr() const;

    // Load and apply constraints from ROS parameters
    void loadConstraintsFromROSParam();

private:
    // Planning scene monitor for managing planning scene updates and interactions
    planning_scene_monitor::PlanningSceneMonitorPtr _psm;

    // Robot description name for identifying the robot model
    std::string _robot_description;

    // ROS NodeHandle for managing ROS communications such as parameters, topics, and services
    ros::NodeHandle _nh;

    // Database connection pointer for interacting with the warehouse database
    warehouse_ros::DatabaseConnection::Ptr _conn;
};

#endif // DB_INITIALIZER_H
