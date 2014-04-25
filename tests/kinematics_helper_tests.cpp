
#include <KinematicsHelper.h>

using namespace trajectory_planner_moveit;
using namespace std;

void printSolution(const moveit_msgs::RobotState &solution) {

	for(size_t i = 0; i < solution.joint_state.name.size(); ++i) {
		ROS_INFO("%s: %1.4f", solution.joint_state.name[i].c_str(), solution.joint_state.position[i]);
	}
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "kinematics_helper_tests");
	ros::NodeHandle nh;

	geometry_msgs::Pose goal1;

	goal1.position.x = 0.26;
	goal1.position.y = 0.20;
	goal1.position.z = 0.65;
	goal1.orientation.x = 0.755872;
	goal1.orientation.y = -0.612878;
	goal1.orientation.z = -0.0464803;
	goal1.orientation.w = 0.22556;

	geometry_msgs::Pose goal2;

	goal2.position.x = 0.186961;
	goal2.position.y = 0.424272;
	goal2.position.z = 0.543895;
	goal2.orientation.x = -0.230403;
	goal2.orientation.y = -0.673347;
	goal2.orientation.z = 0.484887;
	goal2.orientation.w = -0.508336;

	geometry_msgs::Pose goal3;

	goal3.position.x = 0.187241;
	goal3.position.y = 0.491332;
	goal3.position.z = 0.451825;
	goal3.orientation.x = 0.192586;
	goal3.orientation.y = -0.653183;
	goal3.orientation.z = 0.478419;
	goal3.orientation.w = -0.554418;

	KinematicsHelper helper(nh);
	moveit_msgs::RobotState solution;

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "world_link";
	pose.pose = goal1;

	ROS_INFO("Computing test pose 1");
	if(helper.computeIK("right_arm", pose, solution)) {
		printSolution(solution);
		geometry_msgs::Pose pose;
		ROS_INFO("Computing FK result for given solution");
		if(helper.computeFK(solution, "right_arm_7_link", pose)) {
			ROS_INFO("Calculated pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		} else {
			ROS_ERROR("FK computation failed.");
		}
	} else {
		ROS_ERROR("Computation failed");
		return EXIT_FAILURE;
	}

	pose.pose = goal2;
	cout << endl;
	ROS_INFO("Computing test pose 2");
	if(helper.computeIK("right_arm", pose, solution)) {
		printSolution(solution);
		geometry_msgs::Pose pose;
		ROS_INFO("Computing FK result for given solution");
		if(helper.computeFK(solution, "right_arm_7_link", pose)) {
			ROS_INFO("Calculated pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		} else {
			ROS_ERROR("FK computation failed.");
		}
	} else {
		ROS_ERROR("Computation failed");
		return EXIT_FAILURE;
	}

	pose.pose = goal3;
	cout << endl;
	ROS_INFO("Computing test pose 3");
	if(helper.computeIK("right_arm", pose, solution)) {
		printSolution(solution);
		geometry_msgs::Pose pose;
		ROS_INFO("Computing FK result for given solution");
		if(helper.computeFK(solution, "right_arm_7_link", pose)) {
			ROS_INFO("Calculated pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		} else {
			ROS_ERROR("FK computation failed.");
		}
	} else {
		ROS_ERROR("Computation failed");
		return EXIT_FAILURE;
	}

	// try to compute ik solution with collision
	geometry_msgs::Pose goal4;

	goal4.position.x = 0.0;
	goal4.position.y = 0.3;
	goal4.position.z = 0.1;
	goal4.orientation.x = 0.0;
	goal4.orientation.y = 0.0;
	goal4.orientation.z = 0.0;
	goal4.orientation.w = 1.0;

	pose.pose = goal4;

	cout << endl;
	ROS_INFO("Computing test pose with collision");
	if(helper.computeIK("right_arm", pose, solution, true)) {
//		printSolution(solution);
		ROS_ERROR("Computation with collision possible");
		return EXIT_FAILURE;
	} else {
		ROS_INFO("Computation failed as expected");
	}

	cout << endl;
	ROS_INFO("Computing same pose without collision avoidance");
	if(helper.computeIK("right_arm", pose, solution, false)) {
//		printSolution(solution);
		ROS_INFO("Solution found as expected");
	} else {
		ROS_ERROR("Computation failed - no solution found");
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

