/*
 * PlanExecutionNode.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: martin
 */

#include <ros/ros.h>

#include <definitions/TrajectoryExecution.h>
#include <definitions/TrajectoryPlanning.h>

#include <PlanningHelper.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define DEF_PLAN_ATTEMPTS 2
#define DEF_MAX_PLAN_TIME 10

#define GOAL_JOINT_TOLERANCE 1e-4
#define GOAL_POS_TOLERANCE 1e-3 //1 mm
#define GOAL_ORIENT_TOLERANCE 1e-3 //~1 deg

#define PLANNING_SERVICE_NAME "/trajectory_planner_srv"
#define EXECUTION_SERVICE_NAME "/trajectory_execution_srv"

using namespace definitions;
using namespace moveit_msgs;
using namespace ros;
using namespace std;

/**
 * Rosnode that provides the services for planning and execution of robot motions
 */
class PlanExecution {

private:

	// holds the current planning instance - right_arm or left_arm
	PlanningHelperPtr _planning_helper;

	// caches all planned trajectories for later execution
	PlanningResultPtr _motion_plan;

	ServiceServer _planning_server;
	ServiceServer _execution_server;

	bool _initialized;
	bool _executing;

	boost::mutex _mutex;

	/**
	 * Convenience method to fill in the trajectory data from the given PlanningResult into
	 * the given Trajectory
	 *
	 * @param plan
	 * @param trajectory
	 */
	void PlanningResultToTrajectory(const PlanningResultPtr result, vector<Trajectory> &trajectories) {
		for(size_t i = 0; i < result->trajectory_stages.size(); ++i) {
			moveit_msgs::RobotTrajectory &stage = result->trajectory_stages[i];
			definitions::Trajectory trajectory;

			const vector<trajectory_msgs::JointTrajectoryPoint> &points = stage.joint_trajectory.points;
			// pick each point from the trajectory and create a UIBKRobot object
			for (size_t i = 0; i < points.size(); ++i) {
				definitions::UIBKRobot robot_point;
				robot_point.arm.joints.assign(points[i].positions.begin(), points[i].positions.end());
				trajectory.robot_path.push_back(robot_point);
			}

			trajectories.push_back(trajectory);
		}
	}
	/**
	 * Convenience method to fill in the trajectory data from the given SDHand into
	 * the given Trajectory
	 *
	 * @param t the trajectory message to fill
	 * @param hand the SDHand where the data comes from.
	 */
	void fillTrajectory(trajectory_msgs::JointTrajectory &t, const definitions::SDHand &hand) {

		t.header.frame_id = "world_link";
		t.header.stamp = ros::Time::now();
		// Name of joints:
		t.joint_names.push_back("right_sdh_knuckle_joint");
		t.joint_names.push_back("right_sdh_finger_12_joint");
		t.joint_names.push_back("right_sdh_finger_13_joint");
		t.joint_names.push_back("right_sdh_finger_22_joint");
		t.joint_names.push_back("right_sdh_finger_23_joint");
		t.joint_names.push_back("right_sdh_thumb_2_joint");
		t.joint_names.push_back("right_sdh_thumb_3_joint");
		// Position of joints
		trajectory_msgs::JointTrajectoryPoint point;

		point.positions.push_back(hand.joints[0]);
		point.positions.push_back(hand.joints[1]);
		point.positions.push_back(hand.joints[2]);
		point.positions.push_back(hand.joints[3]);
		point.positions.push_back(hand.joints[4]);
		point.positions.push_back(hand.joints[5]);
		point.positions.push_back(hand.joints[6]);

		point.time_from_start = ros::Duration(2);

		t.points.push_back(point);

	}
	/**
	 * Compute gripper translation from given start and goal poses.
	 *
	 * @param start The start position of the gripper
	 * @param goal The goal position of the gripper
	 * @param translation The GripperTranslation message to fill
	 */
	void computeTranslation(const geometry_msgs::Pose &start,
							const geometry_msgs::Pose &goal,
							moveit_msgs::GripperTranslation &translation) {

		Eigen::Vector3d start_vec(start.position.x, start.position.y, start.position.z);
		Eigen::Vector3d goal_vec(goal.position.x, goal.position.y, goal.position.z);

		Eigen::Vector3d t_vec = goal_vec - start_vec;
		double distance = t_vec.norm();
		t_vec.normalize();

		translation.direction.vector.x = t_vec.x();
		translation.direction.vector.y = t_vec.y();
		translation.direction.vector.z = t_vec.z();

		translation.desired_distance = distance;
		// just to give a little bit of flexibility here...
		translation.min_distance = distance * 0.7;
		translation.direction.header.frame_id = "world_link";
		translation.direction.header.stamp = ros::Time::now();

	}
	/**
	 * Convert given definitions::Grasp message into corresponding moveit_msgs::Grasp message with
	 * given grasp_id
	 *
	 * @param grasp_id The name of the resulting grasp
	 * @param d_grasp The definintions::Grasp message where the data comes from
	 * @param m_grasp The moveit_msgs::Grasp message to fill
	 */
	void DefGraspToMoveitGrasp(const string &grasp_id, const definitions::Grasp &d_grasp, moveit_msgs::Grasp m_grasp) {

		trajectory_msgs::JointTrajectory pre_grasp_posture;
		fillTrajectory(pre_grasp_posture, d_grasp.grasp_trajectory[0]);

		trajectory_msgs::JointTrajectory grasp_posture;
		fillTrajectory(grasp_posture, d_grasp.grasp_trajectory[2]);

		geometry_msgs::Pose grasp_pose = d_grasp.grasp_trajectory[2].wrist_pose.pose;
		geometry_msgs::Pose pre_grasp_pose = d_grasp.grasp_trajectory[0].wrist_pose.pose;

		GripperTranslation pre_grasp_approach;
		computeTranslation(pre_grasp_pose, grasp_pose, pre_grasp_approach);

		GripperTranslation post_grasp_retreat;
		// the retreat translation is the opposite of the approach translation.
		computeTranslation(grasp_pose, pre_grasp_pose, post_grasp_retreat);

		m_grasp.id = grasp_id;
		m_grasp.grasp_quality = 1;
		m_grasp.max_contact_force = 0;
		m_grasp.grasp_pose.header.frame_id = "world_link";
		m_grasp.grasp_pose.header.stamp = ros::Time::now();
		m_grasp.grasp_pose.pose = grasp_pose;
		m_grasp.pre_grasp_posture = pre_grasp_posture;
		m_grasp.grasp_posture = grasp_posture;
		m_grasp.pre_grasp_approach = pre_grasp_approach;
		m_grasp.post_grasp_retreat = post_grasp_retreat;
	}

	/**
	 * Callback method for the trajectory planning service server
	 *
	 * @param request
	 * @param response
	 * @return
	 */
	bool PlanningServiceCB(TrajectoryPlanning::Request &request,
						   TrajectoryPlanning::Response &response) {

		// prevent from planning during execution...
		if(_executing) {
			ROS_ERROR("Unable to plan while moving the robot!");
			return false;
		}
		
		string arm = "right_arm";

		ROS_INFO("Received trajectory planning request.");
		ROS_INFO("Validating received request data...");
		// we have to make sure that sufficient date has been provided to be able to compute a plan...
		if(request.arm.empty()) {
			ROS_WARN("No arm name provided - assuming %s", arm.c_str());
		} else {
			arm = request.arm;
		}

		if(request.ordered_grasp.size() < 3) {
			ROS_ERROR("Not enough grasp data provided. Size of 'request.ordered_grasp' has to be >= 3!");
			response.result = TrajectoryPlanning::Response::OTHER_ERROR;
			return false;
		}

		//TODO: fix that later
		// set the current move_group instance to value based on request.arm parameter
		if(arm == "right_arm") {
//			_move_group = _right_arm;
		} else if(arm == "left_arm") {
//			_move_group = _left_arm;
		} else {
			ROS_ERROR("Unknown planning group '%s'!", arm.c_str());
			response.result = TrajectoryPlanning::Response::OTHER_ERROR;
			return false;
		}

		ros::Time start_time = ros::Time::now();

		// clear previously cached motion plan
		_motion_plan.reset();

		vector<moveit_msgs::Grasp> grasp_list;
		// make our grasp list the same size as the provided ordered grasps...
		grasp_list.resize(request.ordered_grasp.size());

		// lock the mutex during path planning...
		_mutex.lock();

		for(size_t i = 0; i < request.ordered_grasp.size(); ++i) {
			// populate moveit_msgs::Grasp at current indes with our grasp data
			stringstream ss;
			ss << "Grasp" << i;
			string id = ss.str();
			DefGraspToMoveitGrasp(id, request.ordered_grasp[i], grasp_list[i]);
		}
		// now we have a vector of grasps and can invoke the planner...
		PlanningResultPtr result = _planning_helper->plan_pick("", grasp_list);

		_mutex.unlock();

		if(result->status == PlanningHelper::SUCCESS) {
			PlanningResultToTrajectory(result, response.trajectory);
			_motion_plan = result;
			response.result = TrajectoryPlanning::Response::SUCCESS;
		} else {
			response.result = TrajectoryPlanning::Response::NO_FEASIBLE_TRAJECTORY_FOUND;
		}

		ros::Duration duration = ros::Time::now() - start_time;

		ROS_INFO("Trajectory planning request completed");
		ROS_INFO_STREAM("Total trajectory calculation took " << duration);

		return true;
	}

	/**
	 * Callback method for the trajectory execution service server.
	 *
	 * @param request
	 * @param response
	 * @return
	 */
	bool ExecutionServiceCB(TrajectoryExecution::Request &request,
							TrajectoryExecution::Response &response) {

		ROS_INFO("Received trajectory execution request");

		if(request.trajectory.size() == 0) {
			ROS_ERROR("No trajectory provided in execution request");
			response.result = TrajectoryExecution::Response::OTHER_ERROR;
			return false;
		}

		if(!_motion_plan) {
			ROS_ERROR("Unable to execute - no motion plan available!");
			response.result = TrajectoryExecution::Response::OTHER_ERROR;
			return false;
		}

		ROS_INFO("Executing trajectory...");

		// lock the mutex during execution...
		_mutex.lock();
		_executing = true;

		bool success = _planning_helper->execute(_motion_plan);

		_executing = false;
		_mutex.unlock();

		if(!success) {
			ROS_ERROR("Trajectory execution failed!");
			response.result = TrajectoryExecution::Response::OTHER_ERROR;
			return false;
		}

		ROS_INFO("Pose target reached");

		// clear previously cached motion plan because it is not valid any more...
		_motion_plan.reset();
		// everything went fine...
		response.result = TrajectoryExecution::Response::SUCCESS;
		return true;
	}


public:
  
	PlanExecution(): _initialized(false), _executing(false) {
	}

	virtual ~PlanExecution() {}
	/**
	 * Initializes the move_group instance.
	 * Reads various parameters from parameter server and sets
	 * the values accordingly.
	 * @param nh public nodehandle
	 * @param p_nh private nodehandle, used for retrieving parameters
	 */
	bool initialize(NodeHandle &nh, NodeHandle &p_nh) {

		if(_initialized) {
			return true;
		}

//		string group_name = "right_arm";
//		if(!p_nh.getParam("planning_group_name", group_name)) {
//			ROS_WARN("Paramteter 'planning_group_name' not set. Assuming '%s' as default.", group_name.c_str());
//		}

		ROS_INFO("Creating PlanningHelper instance...");
		_planning_helper.reset(new PlanningHelper("right_arm"));

		int attempts = DEF_PLAN_ATTEMPTS;
		if(!p_nh.getParam("num_planning_attempts", attempts)) {
			ROS_WARN("Parameter 'num_planning_attempts' not set. Using default value instead.");
		}
		/*
		 * Comment the following lines, if you face problems during compiling.
		 * In that case i would strongly recommend that you update your version of MoveIt,
		 * because this parameter is important!
		 */
		_planning_helper->setPlanningAttempts(attempts);

		double max_plan_time = DEF_MAX_PLAN_TIME;
		if(!p_nh.getParam("max_planning_time", max_plan_time)) {
			ROS_WARN("Parameter 'max_planning_time' not set. Using default value instead.");
		}
		_planning_helper->setAllowedPlanningTime(max_plan_time);

		string planner_id;
		if(!p_nh.getParam("planner_id", planner_id)) {
			ROS_WARN("Parameter 'planner_id' not set. Using default value instead.");
		}
		ROS_INFO("Using planner '%s'", planner_id.c_str());
		_planning_helper->setPlannerId(planner_id);
		_planning_helper->setSupportSurfaceName("table_surface_link");

		ROS_INFO("Starting services...");

		// start service endpoints. Use public nodehandle for that
		_planning_server = nh.advertiseService(PLANNING_SERVICE_NAME, &PlanExecution::PlanningServiceCB, this);
		_execution_server = nh.advertiseService(EXECUTION_SERVICE_NAME, &PlanExecution::ExecutionServiceCB, this);

		_initialized = true;
		ROS_INFO("Connected!");

		return true;
	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "plan_execution_node");
	AsyncSpinner spinner(1);
	spinner.start();

	ROS_INFO("Launching plan_execution_node");
	// public nodehandle
	ros::NodeHandle nh;
	// private nodehandle for retrieving parameters
	ros::NodeHandle p_nh("~");
	ROS_INFO("Initializing planner"); 

	PlanExecution pe;
	if(!pe.initialize(nh, p_nh)) {
		ROS_ERROR("Unable to start plan execution node!");
		return EXIT_FAILURE;
	}

	ros::spin();

	return EXIT_SUCCESS;
}
