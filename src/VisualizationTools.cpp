
#include <uibk_planning_node/VisualizationTools.h>

using namespace std;

namespace trajectory_planner_moveit {


VisualizationTools::VisualizationTools(ros::NodeHandle &nh)
{
	robot_state_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>("uibk_robot_state", 1, true);
	trajectory_pub_ = nh.advertise<moveit_msgs::DisplayTrajectory>("uibk_robot_trajectory", 1, true);
}

void VisualizationTools::publish_robot_state(const moveit_msgs::RobotState &state)
{
	moveit_msgs::DisplayRobotState msg;
	msg.state = state;
	robot_state_pub_.publish(msg);
}

void VisualizationTools::publish_trajectory(const moveit_msgs::RobotState &start_state,
											const moveit_msgs::RobotTrajectory &trajectory)
{
	vector<moveit_msgs::RobotTrajectory> trajectories;
	trajectories.push_back(trajectory);
	publish_trajectory(start_state, trajectories);
}

void VisualizationTools::publish_trajectory(const moveit_msgs::RobotState &start_state,
											const vector<moveit_msgs::RobotTrajectory> &trajectories)
{
	moveit_msgs::DisplayTrajectory msg;
	msg.model_id = "robot_eddie";
	msg.trajectory_start = start_state;

	for (size_t i = 0; i < trajectories.size(); ++i) {
		msg.trajectory.push_back(trajectories[i]);
	}

	trajectory_pub_.publish(msg);
}

}
