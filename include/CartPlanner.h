//// ros headers 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/kinematic_constraints/utils.h>

//// local headers
#include <definitions/TrajectoryPlanning.h>

namespace trajectory_planner_moveit {

// use a name for the node and a verb it is suppose to do, Publisher, Server, etc...
class CartPlanner
{
  private:

    // clients
    ros::ServiceClient clt_moveit_planning_;

	// the variable where the plans are stored
	std::vector<moveit_msgs::MotionPlanResponse> motion_plans_;
	ros::Subscriber sub_collision_objects_;
	std::vector<moveit_msgs::AttachedCollisionObject> collision_objects_;
   
  public:

  	// planning related parameters
	int max_points_in_trajectory_;
	int max_planning_attempts_;
	int max_planning_time_;
	double tolerance_in_position_;
	double tolerance_in_orientation_;

    // define the names passed in the urdf files corresponding to the current move group for planning
    std::string group_name_;
    std::string base_frame_for_goal_;
	std::string plan_for_frame_;

	// joint state topic
	std::string topic_;

	// speed scale for trajectory
	double speed_;

  	// the service callback 
  	bool planTrajectoryFromCode(definitions::TrajectoryPlanning::Request &request, definitions::TrajectoryPlanning::Response &response);

  	// the actual planning function
    bool planTrajectory(std::vector<definitions::Trajectory> &trajectories, definitions::SDHand &goal, std::string &arm, sensor_msgs::JointState &startState);
    void callback_collision_object(const moveit_msgs::AttachedCollisionObject &object);

    // constructor
    CartPlanner(ros::NodeHandle nh)
    {

		// wait for moveit to load
		std::string planning_service_name = "/plan_kinematic_path";

		ROS_INFO("Waiting for MoveIt! to fully load...");
		ros::service::waitForService(planning_service_name, -1);
		clt_moveit_planning_ = nh.serviceClient<moveit_msgs::GetMotionPlan>(planning_service_name);

		// planning related parameters default
		max_points_in_trajectory_ = 60;
		max_planning_attempts_ = 100;
		max_planning_time_ = 10;
		tolerance_in_position_ = 0.01;
		tolerance_in_orientation_ = 0.01;

		// joint state topic
		topic_ = nh.resolveName("/joint_states");

		// to double the speed of the trajectory
		speed_ = 1.0;
		sub_collision_objects_ = nh.subscribe ("/attached_collision_object", 500, &CartPlanner::callback_collision_object, this);
    }

    //! Empty stub
    ~CartPlanner() {}

};

} // namespace trajectory_planner_moveit
