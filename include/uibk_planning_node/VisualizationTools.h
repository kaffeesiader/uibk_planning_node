#ifndef VISUALIZATIONTOOLS_H
#define VISUALIZATIONTOOLS_H

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>

using namespace std;

namespace trajectory_planner_moveit {

class VisualizationTools {

private:

	ros::Publisher robot_state_pub_;
	ros::Publisher trajectory_pub_;

public:

	VisualizationTools(ros::NodeHandle &nh);
	~VisualizationTools() {}

	void publish_robot_state(const moveit_msgs::RobotState &state);
	void publish_trajectory(const moveit_msgs::RobotState &start_state,
							const moveit_msgs::RobotTrajectory &trajectory);
	void publish_trajectory(const moveit_msgs::RobotState &start_state,
							const vector<moveit_msgs::RobotTrajectory> &trajectories);
};

}



#endif // VISUALIZATIONTOOLS_H
