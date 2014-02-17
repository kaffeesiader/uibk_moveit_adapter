/*
 * FakeControllerManager.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: martin
 */

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <UibkRobotHW.h>
#include <controller_manager/controller_manager.h>
#include <uibk_moveit_adapter/load_configuration.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

using namespace std;
using namespace ros;
using namespace controller_manager;

boost::shared_ptr<UibkRobotHW> hw_;
boost::thread cm_thread;

void controller_manager_thread() {

	ros::Rate r(100);

	ros::Time lastTimeStamp = ros::Time::now();
	ros::Time hwTime;

	// create controller manager instance
	ControllerManager cm(hw_.get());

	ROS_INFO("Entering controller manager callback loop...");

	while (ros::ok()) {
		ros::Time currentTimeStamp = ros::Time::now();
		ros::Duration duration = currentTimeStamp - lastTimeStamp;

		hwTime += duration;

		cm.update(currentTimeStamp, duration);
		hw_->publish();

		lastTimeStamp = currentTimeStamp;
		r.sleep();
	}

	ROS_INFO("Controller manager callback loop shut down");
}

bool load_configuration(NodeHandle &nh) {

	ROS_INFO("loading adapter configuration...");

	hw_.reset(new UibkRobotHW);

	if(!hw_->initialize(nh)) {
		ROS_ERROR("Could not initialize hardware adapter!");
		return false;
	}

	ROS_INFO("Hardware adapter configuration successfully loaded");

	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "uibk_controller_manager");

	// create private Nodehandle for initializing RobotHW
	NodeHandle nh("~");

	ROS_INFO("Launching hardware adapter...");

	// Load adapter configuration
	if(!load_configuration(nh)) {
		ROS_ERROR("Unable to start hardware adapter!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Launching controller manager...");
	// launch controller manager thread
	cm_thread = boost::thread(controller_manager_thread);

	ROS_INFO("Controller manager launched.");

	ros::spin();

	ROS_INFO("Wait for ControllerManager callback loop shutdown.");
	cm_thread.join();

	ROS_INFO("Hardware adapter shutdown completed.");

	return EXIT_SUCCESS;

}

