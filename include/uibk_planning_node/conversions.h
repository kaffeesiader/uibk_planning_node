#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <sstream>
#include <moveit_msgs/RobotState.h>

using namespace std;


double getJointPositionFromState(const string &joint, const moveit_msgs::RobotState &state) {
	double value = 0.0;

	for(size_t i = 0; i < state.joint_state.name.size(); ++i) {

		if(joint == state.joint_state.name[i]) {
			value = state.joint_state.position[i];
			break;
		}

	}
	return value;
}

void getJointPositionsFromState(const vector<string> &joints, const moveit_msgs::RobotState &state, vector<double> &values) {

	for (size_t i = 0; i < joints.size(); ++i) {
		double value = getJointPositionFromState(joints[i], state);
		values.push_back(value);
	}

}

void getArmJointNames(const string &arm, vector<string> &names) {

	for(int i = 0; i < 7; ++i) {
		stringstream ss;
		ss << arm << "_arm_" << i << "_joint";
		names.push_back(ss.str());
	}

}

void getArmValuesFromState(const string &arm, const moveit_msgs::RobotState &state, vector<double> &values) {

	vector<string> joints;
	getArmJointNames(arm,joints);
	getJointPositionsFromState(joints, state, values);

}


#endif // CONVERSIONS_H
