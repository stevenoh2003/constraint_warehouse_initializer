#include "db_initializer.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_constraint");

    // Instantiate the DBInitializer with the robot description
    DBInitializer db_initializer("robot_description", argc, argv);

    // Initialize the DBInitializer, which includes loading constraints from ROS parameters
    if (!db_initializer.initialize())
    {
        ROS_ERROR("Failed to initialize the database and load constraints");
        return 1; // Exit with an error code if initialization fails
    }

    ROS_INFO("Database initialized and constraints loaded successfully.");



    ros::shutdown();
    return 0;
}
