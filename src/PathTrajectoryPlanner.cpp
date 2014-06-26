

#include <uibk_planning_node/PathTrajectoryPlanner.h>
#include <moveit/kinematic_constraints/utils.h>
#include <uibk_planning_node/conversions.h>

namespace trajectory_planner_moveit {

PathTrajectoryPlanner::PathTrajectoryPlanner(ros::NodeHandle &nh) : trajectory_planner_(nh) {

    ROS_INFO("Connecting to planning service...");

    string topic = "compute_cartesian_path";
    planning_client_ = nh.serviceClient<moveit_msgs::GetCartesianPath>(topic);

    // set some default values
    arm_ = "right";
    jump_threshold_ = 0.0;
    eef_step_ = 0.01;
}

bool PathTrajectoryPlanner::plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution)
{
    // execute planning with empty start state
    sensor_msgs::JointState start_state;
    return plan(goal, solution, start_state);
}

bool PathTrajectoryPlanner::plan(const geometry_msgs::Pose &goal,
                             moveit_msgs::MotionPlanResponse &solution,
                             const sensor_msgs::JointState &start_state)
{
    if (!planning_client_.exists()) {
        ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");
        return false;
    }

    ros::Time start_time = ros::Time::now();

    ROS_INFO("Constructing planning service request...");

    moveit_msgs::GetCartesianPathRequest request;

    request.header.frame_id = FRAME_ID;
    request.header.stamp = ros::Time::now();

    request.group_name = arm_ + "_arm";
    request.link_name = arm_ + "_arm_7_link";
    request.jump_threshold = jump_threshold_;
    request.max_step = eef_step_;
    request.avoid_collisions = true;
    request.waypoints.push_back(goal);
    request.start_state.joint_state = start_state;

    ROS_INFO("Initial call to planning service...");

    moveit_msgs::GetCartesianPathResponse response;
    // initialize with default value - gets changed if planning was successful.
    solution.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;

    if(!planning_client_.call(request, response)) {
        ROS_INFO_STREAM("Call to cartesian path planning service failed with status code '" << solution.error_code.val << "'");
    }

    bool success = false;

    if(!(response.fraction < 1.0)) {
        ROS_INFO("Planning successful - cartesian planning service did the job!");
        solution.trajectory_start = response.start_state;
        solution.trajectory = response.solution;
        solution.error_code = response.error_code;
        success = true;
    } else {
        ROS_WARN("Cartesian planner failed - calling default planner.");
        success = trajectory_planner_.plan(goal, solution, start_state);
    }

    ros::Time end_time = ros::Time::now();
    ros::Duration planning_duration = end_time - start_time;
    solution.planning_time = planning_duration.toSec();

    return success;
}


}

