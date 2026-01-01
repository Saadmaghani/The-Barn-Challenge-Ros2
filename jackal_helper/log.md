Todo:
0. Spawning jackal in gazebo_launch.launch (done!)
1. move_base_DWA.launch
2. plugins
3. worlds (First BARN, then DynaBARN)
4. configs: 
    1. verbose.conf & non_verbose.conf
    2. params files one by one


## Dec 15:
1. downloaded all required jackal ros2 humble packages and got a simulation up with the jackal using ros2 launch jackal_helper gazebo_launch.launch.py

*Todo:*
1. next step: verify simulation is working by moving jackal around, seeing rviz output
2. next step: get BARN on the simulation. went through deepseek, the world file is not directly compatible, needs to be updated for gazebo fortress. So I will try to see what is the minimal changes needed to make it work. 
3. next step: get DWA with move_base to work. i.e. this basically requires a launch file to launch the ros2 move base, whatever that is.

## Dec 16:
1. verified jackal is working by making it move with the GUI  (Dec15:todo1)
2. verified that BARN worlds are in part transferrable by loading world_0.world. There are visual issues but the cylinders load up. We need to make sure that collisions are working though. (Dec15:todo2)

*Todo:*
1. next step: transform the_barn_challenge/run.py into ros2 compatible launch file.
2. next step: Dec15:todo3.


## Dec 17:
notes on using Nav2:
- installing nav2 was already done, presumably when we installed ros and/or gazebo
- "By default, Nav2 waits for you to give it an approximate starting position." [1]
- BT = behavior tree
- ROS2 nodes have different life cycles. the nodes start in `unconfigured` state in which it is simply constructed. Then it the launcher transitions it to `inactive` using the `on_configure()` method which sets up all parameters, networking interfaces, and dynamically allocated memory. Then it transitions into `active` using the `on_activate()` method which activates ROS networking interfaces and sets the program states to start processing information. Finally, for shutdown nodes transition to `deactive` during which nodes get cleaned up and shutdown and networking interfaces are deactivated and stop processing information. [2]
- Behavior trees are used to combine different nodes.
- controllers in ros2 = local planners in ros1
- behavior server = recovery behavior

References:
1. https://docs.nav2.org/getting_started/index.html
2. https://docs.nav2.org/concepts/index.html



## Dec 18:
What I need to do is to convert run.py into launch files + nodes. the main experiment runner node basically has the logic to run the experiment. the launch file sets up all the nodes needed for the main experiment runner node, including the main node itself. Regarding looping, I can do one of two things:
1. launch the file with different world_files as arguments to the launch file. 
2. OR the launcher is called once and the main node runs the world files sequentially by replacing and reseting the single gazebo instance with different world files. 
Option 2 is better but harder. 

What I've done:
1. got DWA setup via a nav2 launch file using clearpath_nav2_demo (Dec15:todo3)
2. Figured out what run.py needs to become (Dec16:todo1)

*TODO:*
1. write gazebo_simulation.py to help the main run.py file 
2. write the run.py as a node which runs the experiment and outputs the results too.


## DEC 29

*TODO:*
1. Integrate navigation stack (done)
2. Integrate publishing goal (done)
    `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"`
3. simplify navigation launch files (done)
    1. simplify logging to terminal (tried my best)
4. make navigation faster (similar to ROS1) (done)
5. run benchmark on baseline 
6. write documentation
7. publish as "The BARN Challenge"
8. meet with professor
9. DYNA-BARN