# GROUP 15
Federico Pivotto, 2121720, federico.pivotto@studenti.unipd.it

Fabrizio Genilotti, 2119281, fabrizio.genilotti@studenti.unipd.it

Francesco Boscolo Meneguolo, 2119969, francesco.boscolomeneguolo@studenti.unipd.it

# Simulation commands

## Build procedure
After creating and building the catkin workspace following the instructions provided in the assignment, clone the package `ir2425_group_15` into the `src/` folder of the catkin workspace, then execute the `catkin build` command to build the workspace with the package.

**Note**: the build might require to be built multiple times by alternating `catkin build` and `source ./devel/setup.bash`.

## Simulation run
To execute the simulation, first of all run the following four commands in sequence:

1. **Start the simulation** (cmd console 1):
   
   `roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment1`

2. **Navigation stack** (cmd console 2): 

   `roslaunch tiago_iaslab_simulation navigation.launch`

3. **AprilTag** (cmd console 3):

   `roslaunch tiago_iaslab_simulation apriltag.launch`

4. **Apriltags IDs server** (cmd console 4):

   `rosrun tiago_iaslab_simulation apriltag_ids_generator_node`

Once all the previous commands are running properly, execute the following ones:

 - `roslaunch ir2425_group_15 task.launch`

It is possible to visualize the simulation also through the RViz visualization tool:

- `roslaunch ir2425_group_15 rviz.launch`

# Package organization
## Nodes
The nodes in the package are organized into separate folders, each containing the code for the node and its implemented libraries. These nodes are:

- `start_task_node`: it has the role of the node A. It sends to `TaskStatusAction` server both the path of waypoints and the AprilTags IDs to be found (the ones it received from `ids_generator_node`) as goals. It waits for result and then it prints the detection outcome. In the meantime it receives feedbacks of the robot status.
- `apriltag_detection_node`: this node serves to initialize and execute the `AprilTagsDetectionAction` server, that allows to detect in real-time the AprilTags through the camera.
- `robot_navigation_node`: this node serves to initialize and execute the `RobotNavigationAction` server, that exploits the three navigation subroutines described in the previous section through the `robot_controller_node`.
- `robot_controller_node`: this node serves to initialize and execute the `RobotControllerAction` server, which performs and controls the three navigation subroutines.
- `task_status_node`: it has the role of the node B. This node serves to initialize and execute the 'TaskStatusAction' server, that manages both the AprilTags detection and robot navigation tasks, using the previous three nodes. It keeps receiving feedbacks of the robot status.

## Actions

- `AprilTagsDetectionAction`: action for the detection of the AprilTags during navigation. It has as goal the IDs of AprilTags to be found, and the feedback is composed by the IDs and poses of the AprilTags that TIAGo finds. The result comprises: the target AprilTag IDs with their poses relative to the `map` frame, the same information for the non-target AprilTags, and the final status of the task (succeeded or failed).
- `RobotNavigationAction`: action for TIAGo navigation. It has as goal the path of waypoints, and as feedback the robot state and the waypoint reached. The result is whether the task succeeded or not and the robot final state.
- `RobotControllerAction`: action for TIAGo navigation controller. It has as goal the navigation subroutine to execute and a waypoint. The result is the status of the robot and the reached pose. It has no feedback.
- `TaskStatusAction`: action for the state of the task during navigation. It has as the same goals, feedbacks and results of the first two actions, with an additional flag in the result for the success of the task.

## Configuration files

- `waypoints.yaml`: this file contains the poses of the waypoints with respect to the map frame.

## Launch files

- `task.launch`: it runs the nodes `start_task_node`, `apriltags_detection_node`, `robot_navigation_node`, `robot_controller_node` and `task_status_node`, in order to execute the robot's navigation and AprilTags detection routines.
- `rviz.launch`: it runs a preconfigured RViz setup loading the environment with the relative map and frames of reference.

# Assumptions
We made the following assumptions:

1. the width is constant at most the twice the width of TIAGo;
2. the number of obstacles in the corridor is limited to two if they are positioned across the corridor at the same height (i.e., perpendicular to the corridor along the same line);
3. obstacles are not positioned too much close to the centre of the corridor, since TIAGo would not pass;
4. the corridor navigation subroutine of TIAGo is initialized by positioning the robot at the corridor entrance using a waypoint.