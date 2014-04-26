
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <moveit/kinematic_constraints/utils.h>


namespace trajectory_planner_moveit {

bool TrajectoryPlanner::plan(const geometry_msgs::Pose &goal, moveit_msgs::MotionPlanResponse &solution)
{
	// execute planning with empty start state
	sensor_msgs::JointState start_state;
	return plan(goal, solution, start_state);
}

bool TrajectoryPlanner::plan(const geometry_msgs::Pose &goal,
							 moveit_msgs::MotionPlanResponse &solution,
							 const sensor_msgs::JointState &start_state)
{
	if (!planning_client_.exists()) {
		ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");

		return false;
	}

	moveit_msgs::GetMotionPlanRequest get_mp_request;
	moveit_msgs::MotionPlanRequest &request = get_mp_request.motion_plan_request;

	request.group_name = arm_ + "_arm";
	request.num_planning_attempts = planning_attempts_;
	request.allowed_planning_time = planning_time_;
	request.planner_id = planner_id_;

	geometry_msgs::PoseStamped goal_pose;
	goal_pose.header.frame_id = FRAME_ID;
	goal_pose.header.stamp = ros::Time::now();
	goal_pose.pose = goal;

	string end_effector_link = arm_ + "_arm_7_link";

	ROS_DEBUG("Constructing goal constraints for end effector link '%s'", end_effector_link.c_str());

	moveit_msgs::Constraints c =
			kinematic_constraints::constructGoalConstraints(end_effector_link,	goal_pose,
															goal_position_tolerance_,
															goal_orientation_tolerance_);

	request.goal_constraints.push_back(c);
	request.start_state.joint_state = start_state;

	ROS_DEBUG("Calling planning service...");

	moveit_msgs::GetMotionPlanResponse get_mp_response;

	bool success = planning_client_.call(get_mp_request, get_mp_response);

	if(success) {
		solution = get_mp_response.motion_plan_response;

		if(solution.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
			ROS_DEBUG("Solution found for planning problem .");
			return true;
		} else {
			ROS_WARN("Planning failed for unknown reason");
			return false;
		}


	} else {
		ROS_DEBUG_STREAM("Planning failed with status code '" << solution.error_code.val << "'");
		return false;
	}
}


}
