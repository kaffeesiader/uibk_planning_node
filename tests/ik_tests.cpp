/*
 * ik_tests.cpp
 *
 *  Created on: 09.02.2014
 *      Author: martin
 *
 * Compute IK solutions for various poses provided in a textfile.
 * Read poses from file, try to compute IK and store outcome (success or fail) in result file.
 * Results can then be visualized with the visualize_ik_test_results.cpp node
 */


#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <boost/thread.hpp>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include "moveit/robot_model/robot_model.h"

#define IK_ATTEMPTS 5
#define IK_TIMEOUT 0.1

using namespace std;
using namespace ros;
using namespace Eigen;
using namespace robot_model;

Publisher robot_state_publisher_;

/**
 * This structure holds information about an ik test.
 */
struct TestResult {
	// the pose of this test
	geometry_msgs::Pose pose;
	// value indicating if test was successful
	bool success;
	// holds joint positions if calculation was successful
	vector<double> values;
};

/**
 * Publishes given robot state so that it can be visualized in RViz
 */
void publishRobotState(robot_state::RobotState &state) {

	moveit_msgs::DisplayRobotState msg;
	robot_state::robotStateToRobotStateMsg(state, msg.state);

	ROS_INFO("Publishing robot state.");

	robot_state_publisher_.publish(msg);

	// allow rviz to visualize the current state...
	ros::Duration(0.2).sleep();
}

void visualizeResults(RobotModelPtr model, const string &ik_group, vector<TestResult> &results) {
	// create robot state instance
	robot_state::RobotState state(model);
	state.setToDefaultValues();
	// connect to ik group
	const robot_model::JointModelGroup *jnt_model_group = state.getJointModelGroup(ik_group);

	// publish all successful calculated solutions
	for(size_t i = 0; i < results.size(); ++i) {
		if(results[i].success) {
			state.setJointGroupPositions(jnt_model_group, results[i].values);
			publishRobotState(state);
		}
	}
}
/**
 * Write pose and test result into our result file
 */
bool write_results(const char *fn, vector<TestResult> results) {

	ROS_INFO("Writing results to file '%s'", fn);

	ofstream output_file(fn);
	if(!output_file.is_open()) {
		ROS_ERROR("Unable to open output file '%s' for writing!", fn);
		return false;
	}

	// write details into output file
	for(size_t i = 0; i < results.size(); ++i) {
		geometry_msgs::Pose &p = results[i].pose;

		output_file << p.position.x << " " << p.position.y << " " << p.position.z << " ";
		output_file << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w << " ";
		output_file << results[i].success;
		// write the joint positions
		for(size_t j = 0; j < results[i].values.size(); j++) {
			output_file << " " << results[i].values[j];
		}
		output_file << endl;
	}

	return true;
}
/**
 *
 */
void worker(RobotModelPtr model,
			const string &ik_group,
			vector<geometry_msgs::Pose> &test_poses,
			TestResult *results,
			int start_index,
			int n) {

	ROS_INFO("Computing from index %d to %d.", start_index, start_index + n-1);
	// create robot state instance
	robot_state::RobotState state(model);
	state.setToDefaultValues();
	// connect to ik group
	const robot_model::JointModelGroup *jnt_model_group = state.getJointModelGroup(ik_group);
	if(jnt_model_group == NULL) {
		ROS_ERROR("Unknown IK-Group '%s'", ik_group.c_str());
		return;
	}

	ROS_INFO("Starting computation...");
	for(int i = start_index; i < start_index + n; ++i) {
		results[i].pose = test_poses[i];

		// reset state in each iteration
		// comment out if you want to take previous result as seed state
		state.setToDefaultValues();

		// compute result
		if(state.setFromIK(jnt_model_group, test_poses[i], IK_ATTEMPTS, IK_TIMEOUT)) {
			results[i].success = true;
			// store joint positions for later visualitation.
			state.copyJointGroupPositions(jnt_model_group, results[i].values);
			ROS_DEBUG("Result %d: successful", i);
		} else {
			ROS_DEBUG("Result %d: failed", i);
			results[i].success = false;
		}
	}

	ROS_INFO("Worker thread completed.");
}

/**
 * Compute the results for given ik_group and store the outcome in the given vector.
 * If visualize_results is set to true, each solution will be published to rviz,
 * but this option significantly slows down the application!
 */
bool compute_results(string &ik_group, vector<geometry_msgs::Pose> &test_poses,
					 vector<TestResult> &results, bool visualize_results = false)
{
	/* Load the robot model */
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	/* Get a shared pointer to the model */
	// instance of our robot model loaded from URDF
	ROS_INFO("Loading robot model from URDF");
	RobotModelPtr model = robot_model_loader.getModel();

	// check how many CPU's are available and distribute work accordingly
	int numCPU = sysconf( _SC_NPROCESSORS_ONLN );
	int problem_size = test_poses.size();
	int n = problem_size / numCPU;
	int start_index = 0;

	TestResult test_results[test_poses.size()];

	boost::thread_group group;

	// start the computation on all CPU's
	ROS_INFO("Starting computation on %d threads", numCPU);
	for(int i = 0; i < numCPU; ++i) {
		ROS_INFO("Starting worker thread %d", i);
		// the last thread gets all the work left...
		if(i == numCPU - 1) {
			n = problem_size - start_index;
		}

		group.add_thread(new boost::thread(worker, model, ik_group, test_poses, test_results, start_index, n));
		start_index += n;
	}
	ROS_INFO("Waiting for workers to complete...");
	// wait until all threads completed work
	group.join_all();

	int positive_results = 0;

	// put results into resultvector and count positive results
	for(size_t i = 0; i < test_poses.size(); ++i) {
		results.push_back(test_results[i]);
		if(test_results[i].success) {
			positive_results++;
		}
	}

	ROS_INFO("%d IK-solutions found for %d test poses.", positive_results, (int)test_poses.size());

	if(visualize_results) {
		ROS_INFO("Visualizing results");
		visualizeResults(model, ik_group, results);
	}

	return true;
}
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
/**
 * Main entry point
 */
int main(int argc, char **argv) {

	if(argc < 4) {
		ROS_ERROR("Usage: %s INPUT_FN OUTPUT_FN IK_GROUP [optional VISUALIZE_SOLUTIONS]", argv[0]);
		return EXIT_FAILURE;
	}

	char *input_fn = argv[1];
	char *output_fn = argv[2];
	string ik_group = argv[3];

	bool visualize = false;
	if(argc > 4) {
		// try to read visualization paramter
		stringstream ss(argv[4]);
		ss >> visualize;
	}

	ros::init(argc, argv, "ik_tests");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	robot_state_publisher_ = nh.advertise<moveit_msgs::DisplayRobotState>("uibk_robot_state", 1, true);

	ROS_INFO("Starting test run for input file '%s' on IK-group '%s'", input_fn, ik_group.c_str());
	ROS_INFO("Visualization: %s", visualize ? "enabled": "disabled");

	// compute a set of test poses
	vector<geometry_msgs::Pose> test_poses;
	if(!load_test_poses(input_fn, test_poses)) {
		ROS_ERROR("Loading test poses failed!");
		return EXIT_FAILURE;
	}

	// compute the results
	vector<TestResult> results;
	if(!compute_results(ik_group, test_poses, results, visualize)) {
		ROS_ERROR("Result computation failed!");
		return EXIT_FAILURE;
	}

	// write output file
	if(!write_results(output_fn, results)) {
		ROS_ERROR("Writing output file failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Test run completed.");
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
