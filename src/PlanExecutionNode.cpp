/*
 * PlanExecutionNode.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: martin
 */

#include <ros/ros.h>

#include <definitions/TrajectoryExecution.h>
#include <definitions/TrajectoryPlanning.h>

#include <moveit/move_group_interface/move_group.h>

#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<move_group_interface::MoveGroup::Plan> MotionPlanPtr;

#define DEF_PLAN_ATTEMPTS 2
#define DEF_MAX_PLAN_TIME 10

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

	MoveGroupPtr _move_group;
	// caches all planned trajectories for later execution
	vector<MotionPlanPtr> _motion_plans;

	ServiceServer _planning_server;
	ServiceServer _execution_server;

	bool _initialized;

	/**
	 * Convenience method to fill in the trajectory data from the given MotionPlanResponse into
	 * the given Trajectory
	 *
	 * @param plan
	 * @param trajectory
	 */
	void MotionPlanToTrajectory(const move_group_interface::MoveGroup::Plan &plan, Trajectory &trajectory) {
		const vector<trajectory_msgs::JointTrajectoryPoint> &points = plan.trajectory_.joint_trajectory.points;

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
	bool planTrajectory( geometry_msgs::PoseStamped &goal, vector<Trajectory> &trajectories) {

		ROS_INFO("Setting pose target");

		if(!_move_group->setPoseTarget(goal)) {
			ROS_ERROR("Setting pose target failed!");
			return false;
		}

		ROS_INFO("Calling planning service");
		move_group_interface::MoveGroup::Plan plan;
		if(!_move_group->plan(plan)) {
			ROS_WARN("No solution found");
			return false;
		}
		ROS_INFO("Motion plan calculated.");
		int trajSize = (int)plan.trajectory_.joint_trajectory.points.size();

		ROS_INFO("Planning successful completed in %.2fs", plan.planning_time_);
		ROS_INFO("Trajectory contains %d points.", trajSize);
		// store the motion plan for later usage (take index as id)
		int index = _motion_plans.size();
		MotionPlanPtr motion_plan_ptr(new move_group_interface::MoveGroup::Plan(plan));
		_motion_plans.push_back(motion_plan_ptr);

		Trajectory trajectory;
		trajectory.trajectory_id = index;
		// populate trajectory with motion plan data
		MotionPlanToTrajectory(plan, trajectory);

		trajectories.push_back(trajectory);
		return true;

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
		_motion_plans.clear();
		vector<Trajectory> &trajectories = response.trajectory;

		for(size_t i = 0; i < request.ordered_grasp.size(); ++i) {

			geometry_msgs::PoseStamped &goal = request.ordered_grasp[i].grasp_trajectory[0].wrist_pose;
			// ensure that the frame id is set to the world reference frame
			goal.header.frame_id = "world_link";

			if(!planTrajectory(goal, trajectories)) {
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
			response.result = TrajectoryExecution::Response::OTHER_ERROR;
			return false;
		}

		// use first of the trajectories...
		int id = request.trajectory[0].trajectory_id;
		// check if id is valid...
		if(_motion_plans.size() <= (size_t)id) {
			ROS_ERROR("No trajectory available with id '%d'", id);
			response.result = TrajectoryExecution::Response::OTHER_ERROR;
			return false;
		}

		move_group_interface::MoveGroup::Plan &plan = *_motion_plans[id];

		ROS_INFO("Executing trajectory...");

		if(!_move_group->execute(plan)) {
			ROS_ERROR("Trajectory execution failed!");
			response.result = TrajectoryExecution::Response::OTHER_ERROR;
			return false;
		}

		ROS_INFO("Pose target reached");

		// clear all previously cached motion plans because they are not valid any more...
		_motion_plans.clear();
		// everything went fine...
		response.result = TrajectoryExecution::Response::SUCCESS;
		return true;
	}


public:
  
	PlanExecution() : _initialized(false) {
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

		string group_name = "right_arm";
		if(!p_nh.getParam("planning_group_name", group_name)) {
			ROS_WARN("Paramteter 'planning_group_name' not set. Assuming '%s' as default.", group_name.c_str());
		}

		ROS_INFO("Connecting to move_group '%s'", group_name.c_str());
		_move_group.reset(new move_group_interface::MoveGroup(group_name));

		int attempts = DEF_PLAN_ATTEMPTS;
		if(!p_nh.getParam("num_planning_attempts", attempts)) {
			ROS_WARN("Parameter 'num_planning_attempts' not set. Using default value instead.");
		}
		_move_group->setNumPlanningAttempts(attempts);

		double max_plan_time = DEF_MAX_PLAN_TIME;
		if(!p_nh.getParam("max_planning_time", max_plan_time)) {
			ROS_WARN("Parameter 'max_planning_time' not set. Using default value instead.");
		}
		_move_group->setPlanningTime(max_plan_time);

		string planner_id;
		if(!p_nh.getParam("planner_id", planner_id)) {
			ROS_WARN("Parameter 'planner_id' not set. Using default value instead.");
		}
		ROS_INFO("Using planner '%s'", planner_id.c_str());
		_move_group->setPlannerId(planner_id);

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
