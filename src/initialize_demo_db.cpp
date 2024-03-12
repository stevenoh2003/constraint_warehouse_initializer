#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/math/distributions/arcsine.hpp>
#include <ros/ros.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initialize_demo_db", ros::init_options::AnonymousName);

    boost::program_options::options_description desc;
    desc.add_options()("help", "Show help message")("host", boost::program_options::value<std::string>(),
                                                    "Host for the "
                                                    "DB.")("port", boost::program_options::value<std::size_t>(),
                                                           "Port for the DB.");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }
    // Set up db
    warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
    if (vm.count("host") && vm.count("port"))
        conn->setParams(vm["host"].as<std::string>(), vm["port"].as<std::size_t>());
    if (!conn->connect())
        return 1;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
    if (!psm.getPlanningScene())
    {
        ROS_ERROR("Unable to initialize PlanningSceneMonitor");
        return 1;
    }

    // moveit_warehouse::PlanningSceneStorage pss(conn);
    moveit_warehouse::ConstraintsStorage cs(conn);
    // moveit_warehouse::RobotStateStorage rs(conn);
    // pss.reset();
    // cs.reset();
    // rs.reset();

    // add default planning scenes
    // moveit_msgs::PlanningScene psmsg;
    // psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
    // psmsg.name = "default";
    // pss.addPlanningScene(psmsg);
    // ROS_INFO("Added default scene: '%s'", psmsg.name.c_str());

    // moveit_msgs::RobotState rsmsg;
    // moveit::core::robotStateToRobotStateMsg(psm.getPlanningScene()->getCurrentState(), rsmsg);
    // rs.addRobotState(rsmsg, "default");
    // ROS_INFO("Added default state");

    const std::vector<std::string> &gnames = psm.getRobotModel()->getJointModelGroupNames();
    for (const std::string &gname : gnames)
    {
        const moveit::core::JointModelGroup *jmg = psm.getRobotModel()->getJointModelGroup(gname);
        if (!jmg->isChain())
            continue;
        const std::vector<std::string> &lnames = jmg->getLinkModelNames();
        if (lnames.empty())
            continue;

        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = lnames.back();
        ocm.header.frame_id = psm.getRobotModel()->getModelFrame();
        ocm.orientation.x = -0.0;
        ocm.orientation.y = -0.7071068;
        ocm.orientation.z = -0.0;
        ocm.orientation.w = 0.7071068;
        ocm.absolute_x_axis_tolerance = boost::math::constants::pi<double>();
        ocm.absolute_y_axis_tolerance = 0.01;
        ocm.absolute_z_axis_tolerance = 0.01; 
        ocm.weight = 1.0;
        moveit_msgs::Constraints cmsg;
        cmsg.orientation_constraints.resize(1, ocm);
        cmsg.name = ocm.link_name + ":22222";
        cs.addConstraints(cmsg, psm.getRobotModel()->getName(), jmg->getName());
        ROS_INFO("Added default constraint: '%s'", cmsg.name.c_str());
    }
    ROS_INFO("Default MoveIt Warehouse was reset.");

    ros::shutdown();

    return 0;
}