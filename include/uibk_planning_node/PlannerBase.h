#ifndef PLANNERBASE_H
#define PLANNERBASE_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/MotionPlanResponse.h>

using namespace std;

namespace trajectory_planner_moveit {


class PlannerBase {

protected:

    string arm_;

public:

    virtual ~PlannerBase() {}

    virtual const string getName() = 0;
    virtual bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution) = 0;
    virtual bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state) = 0;

    void setArm(const string &arm) { arm_ = arm; }
    string getArm() { return arm_; }

};

}

#endif // PLANNERBASE_H
