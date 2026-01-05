<p align="center">
  <img width = "100%" src='res/BARN_Challenge.png' />
  </p>

--------------------------------------------------------------------------------

# ICRA BARN Navigation Challenge

This is the repository for The BARN Challenge using ROS 2. For ROS 1 see [The BARN Challenge](https://github.com/Daffan/the-barn-challenge).


## Updates:

* 01/01/2026: The BARN Challenge has been updated for ROS2 Jazzy. This is a repository under work. Current limitations (as compared to ROS 1 repo.):
  - No DynaBARN worlds.
  - ~~No Singularity container.~~ (01/04/2026)
  - No plans to support [eband](htps://github.com/utexas-bwi/eband_local_planner.git).
  - Very verbose output.

<!--
* 02/04/2024: Adding 60 [DynaBARN](https://github.com/aninair1905/DynaBARN) environments. DynaBARN environments can be accessed by world indexes from 300-359.
-->

## Requirements
If you run it on a local machine without containers:
* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)


## Installation

#### On local machine
Follow the instructions below to run simulations on your local machines.

1. Create ROS workspace
```
mkdir -p $HOME/jackal_ws/src
cd $HOME/jackal_ws/src
```

2. Clone this repo: 
```
git clone https://github.com/Saadmaghani/The-Barn-Challenge-Ros2
```

3. Install ROS package dependencies: (replace `<YOUR_ROS_VERSION>` with your own, e.g. jazzy. Currently only jazzy is supported)
```
cd ..
source /opt/ros/<YOUR_ROS_VERSION>/setup.bash
rosdep init; rosdep update
rosdep install -y --from-paths . --ignore-src 
```

4. Build the workspace
```
colcon build --symlink-install
```

#### On Singularity container

Follow the instruction below to run simulations in Singularity containers.

1. Follow this instruction to install Singularity: https://docs.sylabs.io/guides/latest/user-guide/quick_start.html. Singularity version = 4.3.0 was used to successfully build the image. Lower versions were not tested.

2. Clone this repo
```
git clone https://github.com/Daffan/the-barn-challenge.git
cd the-barn-challenge
```

3. Build Singularity image (sudo access required)
```
sudo singularity build nav_competition_image.sif Singularityfile.def
```

## Run Simulations

Below is the example to run nav2 with MPPI ([example controller given by clearpath](https://github.com/clearpathrobotics/clearpath_nav2_demos/tree/jazzy)) as the controller.

To run the BARN simulations, simply run the `BARN_runner.launch.py` launch file located in the `jackal_helper` package:
```
source /opt/ros/<YOUR_ROS_VERSION>/setup.bash
source $HOME/jackal_ws/install/local_setup.sh 
ros2 launch jackal_helper BARN_runner.launch.py world_idx:=0
```
Have a look at the [list of arguments](jackal_helper/launch/BARN_runner.launch.py#19) accepted by the launch file. For example, setting `gui:=true` will launch gazebo's gui which, in most cases, is helpful.

To run it in a Singularity container:
```
./singularity_run.sh /path/to/image/file ros2 launch jackal_helper BARN_runner.launch.py world_idx:=0
```


A successful run should print the episode status (collided/succeeded/timeout), the time cost in second, and the navigation metric:
> ```
> >>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
> Navigation succeeded with time 55.6140 (s)
> Navigation metric: 0.1250
> ----------------------------------------------------
> ```

> ```
> >>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
> Navigation collided with time 27.2930 (s)
> Navigation metric: 0.0000
> ----------------------------------------------------
> ```

> ```
> >>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
> Navigation timeout with time 100.0000 (s)
> Navigation metric: 0.0000
> ----------------------------------------------------
> ```


If you run into any issue, please submit a github Issue.

## Test your own navigation stack
We currently don't provide a lot of instructions or a standard API for implementing the navigation stack, but we might add more in this section depending on people's feedback. If you are new to the ROS 2 or mobile robot navigation, we suggest checking [nav2](https://docs.nav2.org/) which provides basic interface to manipulate a robot.

The suggested work flow is to edit the `launch_navigation_stack` method in [BARN_runner.launch.py](jackal_helper/launch/BARN_runner.launch.py#116). Typically, you would write a custom nav2 bringup launch file. We have provided an example launch file [nav2_bringup.launch.py](jackal_helper/launch/nav2_bringup.launch.py). You could also use the provided nav2_bringup launch file while just customizing the [nav2.yaml](jackal_helper/config/nav2.yaml) file. 

You should not edit other parts in BARN_runner.launch.py. 

We provide a bash script `test.sh` to run your navigation stack on 50 uniformly sampled BARN worlds with 10 runs for each world. Once the tests finish, run `python report_test.py --out_path /path/to/out/file` to report the test. 

<!-- Below is an example of MPPI:
```
python report_test.py --out_path res/mppi_out.txt
```
You should see the report as this:
>Avg Time: 33.4715, Avg Metric: 0.1693, Avg Success: 0.8800, Avg Collision: 0.0480, Avg Timeout: 0.0720 -->


## Submission
Submit a link that downloads your customized repository to this [Google form](https://forms.gle/w5s4kV8Xc76s3kMd6). Your navigation stack will be tested in the Singularity container on 50 hold-out BARN worlds sampled from the same distribution as the 300 BARN worlds. In the repository, make sure the `run.py` runs your navigation stack and `Singularityfile.def` installs all the dependencies of your repo. We suggest to actually build an image and test it with `./singularity_run.sh /path/to/image/file ros2 launch jackal_helper BARN_runner.launch.py world_idx:=0`.

