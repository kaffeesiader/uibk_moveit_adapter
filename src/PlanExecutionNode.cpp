/*
 * PlanExecutionNode.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: martin
 */

#include <ros/ros.h>

#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <definitions/TrajectoryExecution.h>
#include <definitions/TrajectoryPlanning.h>

#include <boost/shared_ptr.hpp>
#include <actionlib/client/simple_action_client.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>


#define DEF_PLAN_ATTEMPTS 10
#define DEF_MAX_PLAN_TIME 5.0

#define GOAL_JOINT_TOLERANCE 1e-4
#define GOAL_POS_TOLERANCE 1e-2 //1 mm
#define GOAL_ORIENT_TOLERANCE 1e-2 //~1 deg

#define CAN_LOOK false
#define ALLOW_REPLAN false
#define SUPPORT_SURFACE "table_surface_link"
#define FRAME_ID "world_link"

#define PLANNING_SERVICE_NAME "/trajectory_planner_srv"
#define EXECUTION_SERVICE_NAME "/trajectory_execution_srv"

using namespace definitions;
using namespace moveit_msgs;
using namespace ros;
using namespace std;

struct MotionPlan {
	/// The full starting state used for planning
	moveit_msgs::RobotState start_state_;

	/// The trajectory of the robot (may not contain joints that are the same as for the start_state_)
	moveit_msgs::RobotTrajectory trajectory_;

	/// The amount of time it took to generate the plan
	double planning_time_;
};

typedef boost::shared_ptr<MotionPlan> MotionPlanPtr;

/**
 * Rosnode that provides the services for planning and execution of robot motions
 */
class PlanExecution {

private:

	boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > move_action_client_;
	ros::ServiceClient execution_client_;
	robot_model::RobotModelConstPtr robot_model_;

	// caches all planned trajectories for later execution
	vector<MotionPlanPtr> _motion_plans;

	ServiceServer _planning_server;
	ServiceServer _execution_server;

	bool _initialized;
	bool _executing;

	double planning_time_;
	int planning_attempts_;
	double goal_joint_tolerance_;
	double goal_position_tolerance_;
	double goal_orientation_tolerance_;
	string planner_id_;
	string support_surface_;

	boost::mutex _mutex;

	/**
	 * Convenience method to fill in the trajectory data from the given MotionPlanResponse into
	 * the given Trajectory
	 *
	 * @param plan
	 * @param trajectory
	 */
	void MotionPlanToTrajectory(const MotionPlan &plan, Trajectory &trajectory) {
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
	bool planTrajectory(const string &group_name, geometry_msgs::PoseStamped &pose_goal, vector<Trajectory> &trajectories) {

		ROS_INFO("Setting pose target");

		if (!move_action_client_) {
			ROS_ERROR_STREAM("MoveGroup action client not found");
			return false;
		}
		if (!move_action_client_->isServerConnected()) {
			ROS_ERROR_STREAM("MoveGroup action server not connected");
			return false;
		}
		string end_effector_link = "";
		if(group_name == "right_arm") {
			end_effector_link = "right_arm_7_link";
		} else if(group_name == "left_arm") {
			end_effector_link = "left_arm_7_link";
		} else {
			ROS_WARN("Unknown group name '%s'", group_name.c_str());
		}

		ROS_INFO("Maximum planning time: %.1f, attempts: %d", planning_time_, planning_attempts_);
		moveit_msgs::MoveGroupGoal goal;
		goal.request.group_name = group_name;
		goal.request.num_planning_attempts = planning_attempts_;
		goal.request.allowed_planning_time = planning_time_;
		goal.request.planner_id = planner_id_;
	//	goal.request.workspace_parameters = workspace_parameters_;

		vector<Constraints> constraints;
		Constraints c =	kinematic_constraints::constructGoalConstraints(end_effector_link,	pose_goal,
																		goal_position_tolerance_,
																		goal_orientation_tolerance_);

		goal.planning_options.plan_only = true;
		goal.planning_options.look_around = false;
		goal.planning_options.replan = false;
		goal.planning_options.planning_scene_diff.is_diff = true;
		goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
		goal.request.goal_constraints.push_back(c);

		ROS_INFO("Calling planning service");

		move_action_client_->sendGoal(goal);

		ROS_DEBUG("Sent planning request for pose goal");

		if (!move_action_client_->waitForResult()) {
			ROS_INFO_STREAM("MoveGroup action returned early");
		}

		moveit_msgs::MoveGroupResultConstPtr res = move_action_client_->getResult();

		if (move_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Call to move_group action server succeeded!");
			ROS_INFO("Result: %d", res->error_code.val);

			MotionPlan plan;
			plan.planning_time_ = res->planning_time;
			plan.start_state_ = res->trajectory_start;
			plan.trajectory_ = res->planned_trajectory;

			int trajSize = (int)plan.trajectory_.joint_trajectory.points.size();

			ROS_INFO("Planning successful completed in %.2fs", plan.planning_time_);
			ROS_INFO("Trajectory contains %d points.", trajSize);
			// store the motion plan for later usage (take index as id)
			int index = _motion_plans.size();
			MotionPlanPtr motion_plan_ptr(new MotionPlan(plan));
			_motion_plans.push_back(motion_plan_ptr);

			Trajectory trajectory;
			trajectory.trajectory_id = index;
			// populate trajectory with motion plan data
			MotionPlanToTrajectory(plan, trajectory);

			trajectories.push_back(trajectory);
			return true;

		} else {
			ROS_WARN_STREAM("Fail: " << move_action_client_->getState().toString() << ": " << move_action_client_->getState().getText());
			ROS_WARN_STREAM("Planning failed with status code '" << res->error_code.val << "'");
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

		// prevent from planning during execution...
		if(_executing) {
			ROS_ERROR("Unable to plan while moving the robot!");
			return false;
		}
		// clear all previously calculated motion plans!
		_motion_plans.clear();
		string arm = request.arm;

		ROS_INFO("Received trajectory planning request for '%s'", arm.c_str());

		ros::Time start_time = ros::Time::now();

		// clear all previously cached motion plans
		_motion_plans.clear();
		vector<Trajectory> &trajectories = response.trajectory;

		// lock the mutex during path planning...
		_mutex.lock();

		for(size_t i = 0; i < request.ordered_grasp.size(); ++i) {

			geometry_msgs::PoseStamped &goal = request.ordered_grasp[i].grasp_trajectory[0].wrist_pose;
			// ensure that the frame id is set to the world reference frame
			goal.header.frame_id = "world_link";

			if(!planTrajectory(arm, goal, trajectories)) {
				ROS_WARN("No trajectory found for grasp %d", (int)i);
			}
		}

		_mutex.unlock();

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

		MotionPlan &plan = *_motion_plans[id];

		ROS_INFO("Executing trajectory...");

		ExecuteKnownTrajectory msg;
		ExecuteKnownTrajectoryRequest &erq = msg.request;

		erq.wait_for_execution = true;
		erq.trajectory = plan.trajectory_;

		// lock the mutex during execution...
		_mutex.lock();
		_executing = true;

		bool success = execution_client_.call(msg);

		_executing = false;
		_mutex.unlock();

		if(success) {
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
  
	PlanExecution(): _initialized(false), _executing(false) {

		planning_time_ = 5.0;
		planning_attempts_ = 5;
		goal_joint_tolerance_ = 1e-4;
		goal_position_tolerance_ = 1e-4; // 0.1 mm
		goal_orientation_tolerance_ = 1e-3; // ~0.1 deg
		planner_id_ = "";

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
		/* Load the robot model */
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
		/* Get a shared pointer to the model */
		// instance of our robot model loaded from URDF
		ROS_INFO("Loading robot model from URDF");
		robot_model_ = robot_model_loader.getModel();
		if(!robot_model_) {
			ROS_FATAL_STREAM("Unable to construct robot model!");
			throw runtime_error("Unable to construct robot model!");
		}

		ROS_INFO("Connecting to move_group client...");
		string move_group_topic = "move_group";
		move_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(move_group_topic, true));
		move_action_client_->waitForServer();

		execution_client_ = nh.serviceClient<ExecuteKnownTrajectory>("execute_kinematic_path");

		string id = "";
		p_nh.param("num_planning_attempts", planning_attempts_, DEF_PLAN_ATTEMPTS);
		p_nh.param("max_planning_time", planning_time_, DEF_MAX_PLAN_TIME);
		p_nh.param("planner_id", planner_id_, id);


		goal_joint_tolerance_ = GOAL_JOINT_TOLERANCE;
		goal_position_tolerance_ = GOAL_POS_TOLERANCE;
		goal_orientation_tolerance_ = GOAL_ORIENT_TOLERANCE;

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
