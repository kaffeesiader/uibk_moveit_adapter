/*
 * PlanExecutionNode.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: martin
 */

#include <ros/ros.h>

#include <definitions/TrajectoryExecution.h>
#include <definitions/TrajectoryPlanning.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <moveit/kinematic_constraints/utils.h>
#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr<moveit_msgs::MotionPlanResponse> MotionPlanPtr;

#define MAX_TRAJ_PTS 50
#define PLAN_ATTEMPTS 2
#define MAX_PLAN_TIME 1

using namespace definitions;
using namespace moveit_msgs;
using namespace kinematic_constraints;
using namespace ros;
using namespace std;

ServiceClient planning_client;
ServiceClient execution_client;

ServiceServer planning_server;
ServiceServer execution_server;
// caches all planned trajectories for later execution
vector<MotionPlanPtr> motion_plans;

/**
 * Convenience method to fill in the trajectory data from the given MotionPlanResponse into
 * the given Trajectory
 *
 * @param plan
 * @param trajectory
 */
void MotionPlanToTrajectory(const MotionPlanResponse &plan, Trajectory &trajectory) {
	const vector<trajectory_msgs::JointTrajectoryPoint> &points = plan.trajectory.joint_trajectory.points;

	// pick each point from the trajectory and create a UIBKRobot object
	for (size_t i = 0; i < points.size(); ++i) {
		definitions::UIBKRobot robot_point;
		robot_point.arm.joints.assign(points[i].positions.begin(), points[i].positions.end());
		trajectory.robot_path.push_back(robot_point);
	}
}
/**
 * Plan a trajectory for given pose goal and insert the trajectory into given vector of trajectories.
 * Returns true in case of success. Also store a Pointer to the motion plan for later usage.
 *
 * @param trajectories
 * @param goal
 * @return
 */
bool planTrajectory(vector<Trajectory> &trajectories, geometry_msgs::PoseStamped &goal) {

	GetMotionPlan mp;
	MotionPlanRequest &mp_request = mp.request.motion_plan_request;

	// should not be hardcoded...
	mp_request.group_name = "rightArm";

	// ensure that the frame id of our goal is set to the world reference frame
	goal.header.frame_id = "world_link";

	Constraints pose_goal = constructGoalConstraints("right_arm_7_link", goal, 0.001, 0.001);

	mp_request.goal_constraints.push_back(pose_goal);
	// play with theese values to achieve better performance and effizient motion plans...
	mp_request.num_planning_attempts = PLAN_ATTEMPTS;
	mp_request.allowed_planning_time = MAX_PLAN_TIME;

	ROS_INFO("Calling plannig service...");
	bool success = planning_client.call(mp);

	if(success) {
		MotionPlanResponse &mp_response = mp.response.motion_plan_response;
		int trajSize = (int)mp_response.trajectory.joint_trajectory.points.size();

		ROS_INFO("Planning completed with code %d", mp_response.error_code.val);
		ROS_INFO("Planning took %.2fs", mp_response.planning_time);

		if(trajSize > MAX_TRAJ_PTS) {
			ROS_WARN("Computed trajectory contains to many points, so it will be dropped!");
			return false;
		} else {
			ROS_INFO("Trajectory contains %d points.", trajSize);
			// store the motion plan for later usage
			int index = motion_plans.size();
			MotionPlanPtr motion_plan_ptr(new MotionPlanResponse(mp_response));
			motion_plans.push_back(motion_plan_ptr);

			Trajectory trajectory;
			trajectory.trajectory_id = index;
			// populate trajectory with motion plan data
			MotionPlanToTrajectory(mp_response, trajectory);

			trajectories.push_back(trajectory);
			return true;
		}

	} else {
		ROS_WARN("No solution found");
		return false;
	}
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

	ROS_INFO("Received trajectory planning request");
	ros::Time start_time = ros::Time::now();

	// clear all previously cached motion plans
	motion_plans.clear();
	vector<Trajectory> &trajectories = response.trajectory;

	for(size_t i = 0; i < request.ordered_grasp.size(); ++i) {

		geometry_msgs::PoseStamped &goal = request.ordered_grasp[i].grasp_trajectory[0].wrist_pose;
		// ensure that the frame id is set to the world reference frame
		goal.header.frame_id = "world_link";

		if(!planTrajectory(trajectories, goal)) {
			ROS_WARN("No trajectory found for grasp %d", (int)i);
		}
	}

	if(trajectories.size() > 0) {
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
		return false;
	}
	// use first of the trajectories...
	int id = request.trajectory[0].trajectory_id;

	if(motion_plans.size() <= (size_t)id) {
		ROS_ERROR("No trajectory available with id '%d'", id);
		return false;
	}

	moveit_msgs::ExecuteKnownTrajectory execute_known_trajectory;

	execute_known_trajectory.request.wait_for_execution = true;
	execute_known_trajectory.request.trajectory = motion_plans[id]->trajectory;

	ROS_INFO("Executing trajectory...");
	bool success = execution_client.call(execute_known_trajectory);

	// clear all previously cached motion plans because they are not valid any more...
	motion_plans.clear();

	if (success) {
		MoveItErrorCodes &code = execute_known_trajectory.response.error_code;

		if(code.val == MoveItErrorCodes::SUCCESS) {
			response.result = TrajectoryExecution::Response::SUCCESS;
			ROS_INFO("Execution finished successfully.");
		} else {
			response.result = TrajectoryExecution::Response::OTHER_ERROR;
			ROS_WARN("Execution finished with error_code '%d'", code.val);
		}

	} else {
		ROS_ERROR("Execution failed!");

		return false;
	}

	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "plan_execution_node");
	ros::NodeHandle nh;

	ROS_INFO("Launching plan_executrion_node");

	string planning_service_name = "/plan_kinematic_path";
	string execution_service_name = "/execute_kinematic_path";

	ROS_INFO("Waiting for 'plan_kinematic_path' service node...");
	ros::service::waitForService(planning_service_name, -1);
	planning_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(planning_service_name);

	ROS_INFO("Waiting for 'execute_kinematic_path' service node...");
	ros::service::waitForService(execution_service_name, -1);
	execution_client = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(execution_service_name);

	planning_server = nh.advertiseService("/trajectory_planner_srv", PlanningServiceCB);
	execution_server = nh.advertiseService("/trajectory_execution_srv", ExecutionServiceCB);

	ROS_INFO("Connected!");

	ros::spin();

	return EXIT_SUCCESS;
}

