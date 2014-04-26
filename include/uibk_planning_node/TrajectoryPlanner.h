#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#endif // TRAJECTORYPLANNER_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetMotionPlan.h>

#include <uibk_planning_node/KinematicsHelper.h>


#define CAN_LOOK false
#define ALLOW_REPLAN false
#define FRAME_ID "world_link"

using namespace std;

namespace trajectory_planner_moveit {


class TrajectoryPlanner {

private:

	string arm_;
	double planning_time_;
	int planning_attempts_;
	string planner_id_;
	int max_traj_pts_;
	double goal_joint_tolerance_;
	double goal_position_tolerance_;
	double goal_orientation_tolerance_;

	ros::ServiceClient planning_client_;
	KinematicsHelper kin_helper_;

public:

	TrajectoryPlanner(ros::NodeHandle &nh)
		:kin_helper_(nh)
	{

		ROS_INFO("Connecting to planning service...");

		string topic = "plan_kinematic_path";
		planning_client_ = nh.serviceClient<moveit_msgs::GetMotionPlan>(topic);

		// set some default values
		arm_ = "right";
		planning_time_ = 5.0;
		planning_attempts_ = 5;
		max_traj_pts_ = 50;
		goal_joint_tolerance_ = 1e-4;
		goal_position_tolerance_ = 1e-4; // 0.1 mm
		goal_orientation_tolerance_ = 1e-3; // ~0.1 deg
		planner_id_ = "";

	}

	~TrajectoryPlanner() {}

	void setArm(const string &name) { arm_= name; }
	string getArm() { return arm_; }

	void setPlannerId(const string &planner_id) { planner_id_ = planner_id; }
	string getPlannerId() { return planner_id_; }

	void setAllowedPlanningTime(double value) { planning_time_ = value; }
	double getAllowedPlanningTime() { return planning_time_; }

	void setPlanningAttempts(int value) { planning_attempts_ = value; }
	int getPlanningAttempts() { return planning_attempts_; }

	void setMaxTrajectoryPoints(int value) { max_traj_pts_ = value; }
	int getMaxTrajectoryPoints() { return max_traj_pts_; }

	bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution);
	bool plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state);

};


}
