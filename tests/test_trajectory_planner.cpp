

#include <ros/ros.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <uibk_planning_node/KinematicsHelper.h>


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_traj_planner");
	ros::NodeHandle nh;

	trajectory_planner_moveit::TrajectoryPlanner planner(nh);
	planner.setArm("right");

	trajectory_planner_moveit::KinematicsHelper ki_helper(nh);
	moveit_msgs::MotionPlanResponse plan;

	geometry_msgs::Pose goal1;

	goal1.position.x = 0.26;
	goal1.position.y = 0.20;
	goal1.position.z = 0.65;
	goal1.orientation.x = 0.755872;
	goal1.orientation.y = -0.612878;
	goal1.orientation.z = -0.0464803;
	goal1.orientation.w = 0.22556;

	if(planner.plan(goal1, plan)) {
		ROS_INFO("Plan to goal1 found");
		moveit_msgs::RobotState state;
		geometry_msgs::Pose pose;

		state.joint_state.name = plan.trajectory.joint_trajectory.joint_names;
		state.joint_state.position = plan.trajectory.joint_trajectory.points.back().positions;

		if(ki_helper.computeFK(state, "right_arm_7_link", pose)) {
			ROS_INFO("Target pose:     [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", goal1.position.x, goal1.position.y, goal1.position.z, goal1.orientation.x, goal1.orientation.y, goal1.orientation.z, goal1.orientation.w);
			ROS_INFO("Calculated pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		}
	} else {
		ROS_ERROR("Planning for goal1 failed!");
	}

	geometry_msgs::Pose goal2;

	goal2.position.x = 0.186961;
	goal2.position.y = 0.424272;
	goal2.position.z = 0.543895;
	goal2.orientation.x = -0.230403;
	goal2.orientation.y = -0.673347;
	goal2.orientation.z = 0.484887;
	goal2.orientation.w = -0.508336;

	if(planner.plan(goal2, plan)) {
		ROS_INFO("Plan to goal2 found");
		moveit_msgs::RobotState state;
		geometry_msgs::Pose pose;

		state.joint_state.name = plan.trajectory.joint_trajectory.joint_names;
		state.joint_state.position = plan.trajectory.joint_trajectory.points.back().positions;

		if(ki_helper.computeFK(state, "right_arm_7_link", pose)) {
			ROS_INFO("Target pose:     [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", goal2.position.x, goal2.position.y, goal2.position.z, goal2.orientation.x, goal2.orientation.y, goal2.orientation.z, goal2.orientation.w);
			ROS_INFO("Calculated pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		}
	} else {
		ROS_ERROR("Planning for goal2 failed!");
	}

	geometry_msgs::Pose goal3;

	goal3.position.x = 0.187241;
	goal3.position.y = 0.491332;
	goal3.position.z = 0.451825;
	goal3.orientation.x = 0.192586;
	goal3.orientation.y = -0.653183;
	goal3.orientation.z = 0.478419;
	goal3.orientation.w = -0.554418;

	if(planner.plan(goal3, plan)) {
		ROS_INFO("Plan to goal3 found");
		moveit_msgs::RobotState state;
		geometry_msgs::Pose pose;

		state.joint_state.name = plan.trajectory.joint_trajectory.joint_names;
		state.joint_state.position = plan.trajectory.joint_trajectory.points.back().positions;

		if(ki_helper.computeFK(state, "right_arm_7_link", pose)) {
			ROS_INFO("Target pose:     [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", goal3.position.x, goal3.position.y, goal3.position.z, goal3.orientation.x, goal3.orientation.y, goal3.orientation.z, goal3.orientation.w);
			ROS_INFO("Calculated pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		}
	} else {
		ROS_ERROR("Planning for goal3 failed!");
	}

	return EXIT_SUCCESS;
}

