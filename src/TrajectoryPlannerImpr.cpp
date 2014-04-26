
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <moveit/kinematic_constraints/utils.h>
#include <uibk_planning_node/conversions.h>


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
	request.start_state.joint_state = start_state;

	ROS_DEBUG("Computing possible IK solutions for goal pose");

	vector<string> joint_names;
	getArmJointNames(arm_, joint_names);

	// compute a set of ik solutions and construct goal constraint
	for (int i = 0; i < 5; ++i) {
		moveit_msgs::RobotState ik_solution;

		geometry_msgs::PoseStamped pose_goal;
		pose_goal.header.stamp = ros::Time::now();
		pose_goal.header.frame_id = FRAME_ID;
		pose_goal.pose = goal;

		if(kin_helper_.computeIK(arm_, pose_goal, start_state, ik_solution)) {
			vector<double> values;
			getJointPositionsFromState(joint_names, ik_solution, values);

			moveit_msgs::Constraints c;
			c.joint_constraints.resize(joint_names.size());

			for (int j = 0; j < joint_names.size(); ++j) {
				moveit_msgs::JointConstraint &jc = c.joint_constraints[j];
				jc.joint_name = joint_names[j];
				jc.position = values[j];
				jc.tolerance_above = 1e-4;
				jc.tolerance_below = 1e-4;
				jc.weight = 1.0;
			}
			request.goal_constraints.push_back(c);
		}
	}

	if(request.goal_constraints.size() == 0) {
		ROS_WARN("No valid IK solution found for given pose goal - planning failed!");
		return false;
	}

	ROS_DEBUG("Found %d valid IK solutions for given pose goal", (int)request.goal_constraints.size());
	ROS_DEBUG("Calling planning service...");

	moveit_msgs::GetMotionPlanResponse get_mp_response;

	bool success = planning_client_.call(get_mp_request, get_mp_response);
	solution = get_mp_response.motion_plan_response;

	if(success) {
		int pts_count = (int)solution.trajectory.joint_trajectory.points.size();
		int error_code = solution.error_code.val;

		if(error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
			ROS_DEBUG_STREAM("Planning failed with status code '" << solution.error_code.val << "'");
			return false;
		}

		if(pts_count > max_traj_pts_) {
			ROS_WARN("Valid solution found but contains to many points.");
			return false;
		}

		ROS_DEBUG("Solution found for planning problem .");
		return true;

	} else {
		ROS_DEBUG_STREAM("Planning failed with status code '" << solution.error_code.val << "'");
		return false;
	}
}


}
