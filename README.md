UIBK MoveIt Adapter
===================

This package provides all the necessary infrastructure to interface our current control interface
with the requirements of moveit. It is based on the ros_control stack, that is part of the ROS hydro
distribution. If you use ROS groovy, you have to download the source code from

  https://github.com/ros-controls/ros_control
  https://github.com/ros-controls/ros_controllers
  https://github.com/ros-controls/realtime_tools
  https://github.com/ros-controls/control_toolbox
  
Read the INSTALL_ROS_CONTROL file for further details!

Build the hardware adapter package:

  rosmake uibk_moveit_adapter

After building the 'uibk_moveit_adapter' package, you can launch the hardware adapter via
'roslaunch uibk_moveit_adapter hardware_adapter.launch' for interfacing the real robot, or
'roslaunch uibk_moveit_adapter hardware_adapter_simulation.launch' for interfacing the simulator.

When this node launches without errors everything is ready to interact with MoveIt.

Description
===========

The interface reads from our JointStates topics and publishes to our MoveJoints topics. It also provides
a controller_manager node that can load and run hardware controllers from the ROS control stack (for
example the FollowJointTrajectory Controller that can be used to execute MoveIt generated trajectories).

It is possible to create various configurations (for example one for the simulator and one for the
real robot) and switch between the configurations via a service call.

Switch between configurations via 'rosservice call /fake_controller_manager/load_config [config]'
where config can be 'sim' or 'real'. Topic names can be adjusted in the 'adapter_config.yaml' file

Launch the adapter using the 'hardware_adapter.launch' file for adapting the real robot, and
the 'hardware_adapter_sim.launch' file for adapting the simulator.

In order to work correctly the adapter needs the JointPositions of both arms. So it is necessary that both
arms are running and publishing joint state messages. So ensure that both arms are running and are in
control mode 10 or 30 if you use the real robot. Otherwise the hardware adapter will not work!

CAUTION!!!!
If the robot runs into an error and switches to another control mode, this adapter will keep on publishing
JointPosition messages to the arms. So if the arms move to another position in the meantime and you switch 
back to the control mode 10 or 30, the robot will instantly try to move very fast to the last commanded 
joint position. So please always shut down the adapter node in case of an error and relaunch it when everything
is fine again!!!

UIBK Plan Execution Node
========================

The uibk_plan_execution_node provides the two services: 

  - /trajectory_planner_srv (TrajectoryPlanning.srv)
    Plans trajectories for a given list of grasps. It also caches the planned trajectories for further
    execution.
    
    Note: at the moment the planning service just calculates trajectories to move the arm to the
	  desired wrist-pose. The hand is not considered in this version.
	  
  - /trajectory_execution_srv (TrajectoryExecution.srv)
    Executes a given trajectory (the first one in the list). After the execution all other cached
    Trajectories get deleted because they are not valid any more.
    
This node uses various services provided by MoveIt so it can only work if the MoveGroup node was launched
previously. For launching all necessary nodes you can use the 'uibk_plan_execution.launch' file. This will
also launch the RViz Visualization tools for visualizing the planned trajectories.

As this services are very new and not tested very well I ask you to be very careful! Please execute no trajectoriy
without checking it's visualization in RViz first!

