#ifndef TRAJECTORYTOOLS_H
#define TRAJECTORYTOOLS_H

#include <moveit_msgs/RobotTrajectory.h>
#include <uibk_planning_node/KinematicsHelper.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

namespace trajectory_planner_moveit {

class TrajectoryTools {

public:

    static double computeTrajectoryDistanceRatio(const string &arm,
                                          const moveit_msgs::RobotTrajectory &trajectory,
                                          KinematicsHelper &ki_helper) {

        const trajectory_msgs::JointTrajectory &traj = trajectory.joint_trajectory;
        // trajectory must contain at least 2 points for distance computation
        if(traj.points.size() < 2) {
            return 0.0;
        }

        string link_name = arm + "_arm_7_link";

        // compute the eef position for each point in the trajectory
        vector<geometry_msgs::Pose> positions;
        positions.resize(traj.points.size());
        for(size_t i = 0; i < traj.points.size(); ++i) {
            moveit_msgs::RobotState state;
            state.joint_state.name = traj.joint_names;
            state.joint_state.position = traj.points[i].positions;
            ki_helper.computeFK(state, link_name, positions[i]);
        }

        // compute the direct distance between start and end point
        double direct_dist = computeDistance(positions.front(), positions.back());

        // return 1.0 for trajectories where direct distance is smaller than 5cm to avoid corruption
        // of the overall result.
        if(direct_dist < 0.05) {
            ROS_WARN("Direct distance between start and goal is smaller than 5cm - dropping result!");
            return 1.0;
        }

        // compute the distance from each trajectory point to it's predecessor
        // and sum those distances up
        double sum_distances = 0;

        for(size_t i = 1; i < positions.size(); ++i) {
            sum_distances += computeDistance(positions[i], positions[i-1]);
        }
        double ratio = sum_distances / direct_dist;

        ROS_INFO("Direct distance between start and end: %1.3fm", direct_dist);
        ROS_INFO("Total distance end effector has to travell: %1.3fm", sum_distances);
        ROS_INFO("Trajectory distance ratio: %1.3f", ratio);

        return ratio;
    }

    static double computeDistance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
        Eigen::Vector3d start_vec(pose1.position.x, pose1.position.y, pose1.position.z);
        Eigen::Vector3d goal_vec(pose2.position.x, pose2.position.y, pose2.position.z);

        Eigen::Vector3d t_vec = goal_vec - start_vec;

        return t_vec.norm();
    }

};

}

#endif // TRAJECTORYTOOLS_H
