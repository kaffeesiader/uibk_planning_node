//// ros headers


//// local headers
#include <KinematicsHelper.h>


using namespace std;
using namespace ros;



namespace trajectory_planner_moveit {

bool KinematicsHelper::computeIKInternal(const moveit_msgs::GetPositionIKRequest &request, moveit_msgs::RobotState &solution)
{
	moveit_msgs::GetPositionIKResponse response;

	ROS_DEBUG_NAMED("KinematicsHelper", "Calling IK Service...");

	if(ik_client_.call(request, response)) {

		if(response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
			solution = response.solution;
			ROS_INFO_NAMED("KinematicsHelper", "IK solution successfully calculated");

			return true;
		} else {
			ROS_WARN_NAMED("KinematicsHelper", "IK calculation failed with error code '%d'", response.error_code.val);
			return false;
		}

	} else {
		ROS_ERROR("IK Service call failed! Maybe MoveIt was not launched properly.");
		return false;
	}
}

KinematicsHelper::KinematicsHelper(NodeHandle &nh)
{
	ik_client_ = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
//	ik_client_.waitForExistence();
	fk_client_ = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
	//	fk_client_.waitForExistence();
}

bool KinematicsHelper::computeIK(const string &group_name,
								 const geometry_msgs::PoseStamped &goal,
								 moveit_msgs::RobotState &solution,
								 const bool avoid_collisions,
								 const int attempts,
								 const double timeout)
{
	sensor_msgs::JointState state;

	return computeIK(group_name, goal, state, solution, avoid_collisions, attempts, timeout);
}

bool KinematicsHelper::computeIK(const string &group_name,
								 const geometry_msgs::PoseStamped &goal,
								 const sensor_msgs::JointState &seed_state,
								 moveit_msgs::RobotState &solution,
								 const bool avoid_collisions,
								 const int attempts,
								 const double timeout)
{
	ROS_INFO_NAMED("KinematicsHelper", "IK request received for group '%s'", group_name.c_str());

	moveit_msgs::GetPositionIKRequest request;

	request.ik_request.group_name = group_name;
	request.ik_request.pose_stamped = goal;
	request.ik_request.attempts = attempts;
	request.ik_request.timeout = ros::Duration(timeout);
	request.ik_request.avoid_collisions = avoid_collisions;

	moveit_msgs::RobotState seed;
	seed.joint_state = seed_state;
	request.ik_request.robot_state = seed;

	//	request.ik_request.ik_link_name = link_name;
	//	request.ik_request.constraints = constraints;

	return computeIKInternal(request, solution);
}

bool KinematicsHelper::computeFK(const moveit_msgs::RobotState &state,
								 const string &link_name,
								 geometry_msgs::Pose &solution,
								 const string &frame_id)
{
	ROS_INFO_NAMED("KinematicsHelper", "FK request received for link '%s'.", link_name.c_str());

	moveit_msgs::GetPositionFKRequest request;

	request.header.stamp = ros::Time::now();
	request.header.frame_id = frame_id;
	request.robot_state = state;
	request.fk_link_names.push_back(link_name);

	moveit_msgs::GetPositionFKResponse response;

	ROS_DEBUG_NAMED("KinematicsHelper", "Calling FK Service...");

	if(fk_client_.call(request, response)) {

		if(response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
			ROS_DEBUG_NAMED("KinematicsHelper", "Response contains %d results", (int)response.fk_link_names.size());
			ROS_DEBUG_NAMED("KinematicsHelper", "First link in result set: '%s'", response.fk_link_names[0].c_str());

			solution = response.pose_stamped[0].pose;

			ROS_INFO_NAMED("KinematicsHelper", "FK pose successfully calculated");

			return true;
		} else {
			ROS_WARN_NAMED("KinematicsHelper", "FK calculation failed with error code '%d'", response.error_code.val);
			return false;
		}

	} else {
		ROS_ERROR("IK Service call failed! Maybe MoveIt was not launched properly.");
		return false;
	}
}




}
