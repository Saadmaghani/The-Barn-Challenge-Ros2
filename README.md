<p align="center">
  <img width = "100%" src='res/BARN_Challenge.png' />
  </p>

--------------------------------------------------------------------------------

# ICRA BARN Navigation Challenge

## Updates:
* 01/01/2026: The BARN Challenge has been updated for ROS2 Jazzy. See ROS2 branch. Current limitations of ROS2 

* 02/04/2024: Adding 60 [DynaBARN](https://github.com/aninair1905/DynaBARN) environments. DynaBARN environments can be accessed by world indexes from 300-359.

* 01/01/2026: The BARN Challenge has been updated for ROS2 Jazzy. This is a branch under work. Current limitations of ROS2 branch:
  - No DynaBARN worlds.
  - No Singularity container.
  - [eband](https://github.com/utexas-bwi/eband_local_planner.git) is not currently supported.



## Requirements
If you run it on a local machine without containers:
* ROS version at least Jazzy
* Ubuntu 24.04
* Gazebo Harmonic

The requirements above are just suggestions. If you run into any issue, please contact organizers for help (sghani2@gmu.edu).

## Installation
Follow the instructions below to run simulations on your local machines.

1. Create a virtual environment (we show examples with python venv, you can use conda instead)
```
apt -y update; apt-get -y install python3-venv
python3 -m venv /<YOUR_HOME_DIR>/nav_challenge
export PATH="/<YOUR_HOME_DIR>/nav_challenge/bin:$PATH"
```

2. Install Python dependencies
```
pip3 install catkin_pkg
```

3. Create ROS workspace
```
mkdir -p /<YOUR_HOME_DIR>/jackal_ws/src
cd /<YOUR_HOME_DIR>/jackal_ws/src
```

4. Clone this repo and required ros packages: (replace `<YOUR_ROS_VERSION>` with your own, e.g. melodic)
```
git clone https://github.com/Daffan/the-barn-challenge.git
```

5. Install ROS package dependencies: (replace `<YOUR_ROS_VERSION>` with your own, e.g. melodic)
```
cd ..
source /opt/ros/<YOUR_ROS_VERSION>/setup.bash
rosdep init; rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro=<YOUR_ROS_VERSION>
```

6. Build the workspace (if `catkin_make` fails, try changing `-std=c++11` to `-std=c++17` in `jackal_helper/CMakeLists.txt` line 3)
```
colcon build --symlink-install
source devel/setup.bash
```

## Run Simulations

Below is the example to run move_base with MPPI ([example controller given by clearpath](https://github.com/clearpathrobotics/clearpath_nav2_demos/tree/jazzy)) as local planner.

To run the BARN simulations, simply run the launch file located under jackal_helper/launch/BARN_runner.launch.py:
```
source ../../devel/setup.sh
ros2 launch jackal_helper BARN_runner.launch.py world_idx:=0
```

A successful run should print the episode status (collided/succeeded/timeout) and the time cost in second:
> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation collided with time 27.2930 (s)

> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation succeeded with time 29.4610 (s)


> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
>Navigation timeout with time 100.0000 (s)

## Test your own navigation stack
We currently don't provide a lot of instructions or a standard API for implementing the navigation stack, but we might add more in this section depending on people's feedback. If you are new to the ROS or mobile robot navigation, we suggest checking [move_base](http://wiki.ros.org/move_base) which provides basic interface to manipulate a robot.

The suggested work flow is to edit section 1 in `run.py` file (line 89-109) that initialize your own navigation stack. You should not edit other parts in this file. We provide a bash script `test.sh` to run your navigation stack on 50 uniformly sampled BARN worlds with 10 runs for each world. Once the tests finish, run `python report_test.py --out_path /path/to/out/file` to report the test. Below is an example of DWA:
```
python report_test.py --out_path res/dwa_out.txt
```
You should see the report as this:
>Avg Time: 33.4715, Avg Metric: 0.1693, Avg Success: 0.8800, Avg Collision: 0.0480, Avg Timeout: 0.0720

Except for `DWA`, we also provide three learning-based navigation stack as examples (see branch `LfH`, `applr` and `e2e`).

## Submission
Submit a link that downloads your customized repository to this [Google form](https://docs.google.com/forms/d/e/1FAIpQLSfZLMVluXE-HWnV9lNP00LuBi3e9HFOeLi30p9tsHUViWpqrA/viewform). Your navigation stack will be tested in the Singularity container on 50 hold-out BARN worlds sampled from the same distribution as the 300 BARN worlds. In the repository, make sure the `run.py` runs your navigation stack and `Singularityfile.def` installs all the dependencies of your repo. We suggest to actually build an image and test it with `./singularity_run.sh /path/to/image/file python3 run.py --world_idx 0`. You can also refer to branch `LfH`, `applr` and `e2e`, which are in the correct form for submissions.
