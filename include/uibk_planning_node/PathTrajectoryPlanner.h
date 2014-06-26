#ifndef PATHTRAJECTORYPLANNER_H
#define PATHTRAJECTORYPLANNER_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <uibk_planning_node/PlannerBase.h>


#define FRAME_ID "world_link"

using namespace std;

namespace trajectory_planner_moveit {


class PathTrajectoryPlanner : public PlannerBase{

private:

    double jump_threshold_;
    double eef_step_;
    TrajectoryPlanner trajectory_planner_;

    ros::ServiceClient planning_client_;


public:

    PathTrajectoryPlanner(ros::NodeHandle &nh);
    ~PathTrajectoryPlanner() {}

    void setJumpThreshold(double value) { jump_threshold_ = value; }
    double setJumpThreshold() { return jump_threshold_; }

    void setEEFStep(double value) { eef_step_ = value; }
    double getEEFStep() { return eef_step_; }

    const string getName() { return "PathTrajectoryPlanner"; }

    bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution);
    bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state);

};


}


#endif // PATHTRAJECTORYPLANNER_H
