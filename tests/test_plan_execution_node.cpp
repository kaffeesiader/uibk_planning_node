/*
 * test_plan_execution_node.cpp
 *
 *  Created on: Feb 21, 2014
 *      Author: Martin Griesser
 */


/*
 * test_poses.cpp
 *
 *  Created on: Feb 19, 2014
 *      Author: Martin Griesser
 */


#include "ros/ros.h"
#include "iostream"
#include "fstream"

#include "definitions/TrajectoryExecution.h"
#include "definitions/TrajectoryPlanning.h"

#include "boost/shared_ptr.hpp"

#define PLAN_ATTEMPTS 3

using namespace definitions;
using namespace ros;
using namespace std;

ServiceClient planning_client;
ServiceClient execution_client;

/**
 * Loads the pose data from file with given name into given vector.
 */
bool load_test_poses(const char *fn, vector<geometry_msgs::Pose> &test_poses) {

	ROS_INFO("Opening input file '%s'", fn);

	ifstream input_file(fn);
	if(!input_file.is_open()) {
		ROS_ERROR("Unable to open input file '%s' for reading!", fn);
		return false;
	}

	string line;
	while(getline(input_file, line)) {
		geometry_msgs::Pose p;
		stringstream ss(line);

		ss >> p.position.x >> p.position.y >> p.position.z;
		ss >> p.orientation.x >> p.orientation.y >> p.orientation.z >> p.orientation.w;

		test_poses.push_back(p);
	}

	input_file.close();
	ROS_INFO("%d test poses loaded.", (int)test_poses.size());

	return true;
}

void run_tests(const string &arm, vector<geometry_msgs::Pose> &poses) {
	ROS_INFO("Received %d test poses. Starting tests.", (int)poses.size());

	int successful = 0;
	vector<int> succ;

	for(size_t i = 0; i < poses.size(); ++i) {
		TrajectoryPlanning planning_srv;
		planning_srv.request.arm = arm;
		planning_srv.request.ordered_grasp.resize(1);

		int attempts = 1;
		int pose_target_id = (int)i + 1;

		SDHand hand;
		hand.wrist_pose.pose = poses[i];

		planning_srv.request.ordered_grasp[0].grasp_trajectory.push_back(hand);
		ROS_INFO("Planning for test pose %d", pose_target_id);
		geometry_msgs::Pose &p = poses[i];
		ROS_INFO("Position (%.2f,%.2f,%.2f)", p.position.x, p.position.y, p.position.z);
		ROS_INFO("Orientation(%.2f,%.2f,%.2f,%.2f)", p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);


		while(attempts <= PLAN_ATTEMPTS) {
			ROS_INFO("Starting planning attempt %d", attempts);
			ROS_INFO("Calling planning service");

			planning_client.call(planning_srv);

			if(planning_srv.response.result == TrajectoryPlanning::Response::SUCCESS) {

				ROS_INFO("Motion plan calculated - executing trajectory.");
				TrajectoryExecution execution_srv;
				execution_srv.request.trajectory = planning_srv.response.trajectory[0];

				execution_client.call(execution_srv);
				if(execution_srv.response.result == TrajectoryExecution::Response::SUCCESS) {
					ROS_INFO("Pose target %d reached.\n", pose_target_id);
					succ.push_back(pose_target_id);
					successful++;
					sleep(3);
					break;
				} else {
					ROS_ERROR("Trajectory execution for pose target %d failed!\n", pose_target_id);
				}


			} else {
				ROS_ERROR("Planning for pose target %d failed!\n", pose_target_id);
			}

			attempts++;
		}
	}

	ROS_INFO("Test run completed! %d positions reached, %d failed!", successful, (int)poses.size() - successful);
	for (size_t i = 0; i < succ.size(); ++i) {
		cout << succ[i] << " ";
	}
	cout << endl;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_poses");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	if(argc < 2) {
		ROS_ERROR("Usage: %s INPUT_FILE [optional PLANNING_GROUP_NAME]", argv[0]);
		return EXIT_FAILURE;
	}

	// load test poses from given input file
	vector<geometry_msgs::Pose> test_poses;
	if(!load_test_poses(argv[1], test_poses)) {
		ROS_ERROR("Unable to load test poses from file '%s'", argv[1]);
		return EXIT_FAILURE;
	}

	// try to extract the planning group name from parameters
	// use 'rightArm' as default
	// possible values 'rightArm' and 'leftArm'
	string planning_group_name = "right_arm";
	if(argc > 2) {
		planning_group_name = argv[2];
	}

	ROS_INFO("Connecting to planning and execution services");
	planning_client = nh.serviceClient<definitions::TrajectoryPlanning>("trajectory_planner_srv");
	execution_client = nh.serviceClient<definitions::TrajectoryExecution>("trajectory_execution_srv");

	ROS_INFO("Connected!");

	run_tests(planning_group_name, test_poses);

	return EXIT_SUCCESS;

}
