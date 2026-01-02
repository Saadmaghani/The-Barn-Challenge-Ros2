#!/usr/bin/env python3

import time
import os
from os.path import dirname
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
# from rclpy.executors import SingleThreadedExecutor --> cant make executors inside nodes. meant to be outside nodes.
from ament_index_python.packages import get_package_share_directory

from gazebo_simulation import GazeboSimulationController

# this file will be used to run the experiments and control the simulation via gazebo_simulation

# TODO: \r print statements not being shown 

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)

class BARNRunner(Node):
    def __init__(self):
        super().__init__('BARN_Runner')
        self.get_logger().info("BARN RUNNER started")

        # declare parameters
        self._declare_parameters()

        # initialize 
        self._setup_initial_goal_positions()
        self.timeout = self.get_parameter('timeout').value
        self.file_lock = threading.Lock()

        # start up GazeboSimulation Node
        self.gazebo_sim_ctrl = GazeboSimulationController(init_position=(self.init_coor[0], self.init_coor[1], self.init_yaw_angle))
      
    def reset_jackal_to_start(self):
        self.get_logger().info("Resetting Jackal...")
        
        pos = self.gazebo_sim_ctrl.get_model_state().position
        curr_coor = (pos.x, pos.y)
        collided = True

        while compute_distance(self.init_coor, curr_coor) > 0.1 or collided:
            rclpy.spin_until_future_complete(self, self.gazebo_sim_ctrl.reset_model())
            time.sleep(0.01) # so CPU is not overloaded
            pos = self.gazebo_sim_ctrl.get_model_state().position
            curr_coor = (pos.x, pos.y)
            collided = self.gazebo_sim_ctrl.get_hard_collision()
        
        self.get_logger().info(f"initial position: ({curr_coor[0]:.2f}, {curr_coor[1]:.2f})")
        self.get_logger().info(f"goal position: ({self.goal_coor[0]:.2f}, {self.goal_coor[1]:.2f})")

    def run_trial(self):
        self.get_logger().info(f"Waiting for robot to start moving...")
        curr_time = self.get_clock().now()
        pos = self.gazebo_sim_ctrl.get_model_state().position
        curr_coor = (pos.x, pos.y)

        # check whether the robot started to move
        while compute_distance(self.init_coor, curr_coor) < 0.1:
            # self.get_logger().info(f"x: {curr_coor[0]:.2f} (m), y: {curr_coor[1]:.2f} (m)")
            curr_time = self.get_clock().now()
            pos = self.gazebo_sim_ctrl.get_model_state().position
            curr_coor = (pos.x, pos.y)
            rclpy.spin_once(self.gazebo_sim_ctrl, timeout_sec=0.01) # spin gazebo_sim_ctrl node instead of self so gazebo_sim_ctrl gets updated. or TODO use executor to spin both.
            time.sleep(0.01) # so CPU is not overloaded

        self.get_logger().info(f"Trial running...")
        # start navigation, check position, time and collision
        start_time = curr_time
        start_time_cpu = time.time()
        collided = False
        elapsed = (curr_time - start_time).nanoseconds/1e9

        while compute_distance(self.goal_coor, curr_coor) > 1 and not collided and elapsed < self.timeout:
            curr_time = self.get_clock().now()
            pos = self.gazebo_sim_ctrl.get_model_state().position
            curr_coor = (pos.x, pos.y)
            elapsed = (curr_time - start_time).nanoseconds/1e9
            print(f"Time: {elapsed:.2f} (s), x: {curr_coor[0]:.2f} (m), y: {curr_coor[1]:.2f} (m)", end='\r')
            collided = self.gazebo_sim_ctrl.get_hard_collision()
            rclpy.spin_once(self.gazebo_sim_ctrl, timeout_sec=0.01)
            time.sleep(0.01) # so CPU is not overloaded
            
        print() # so that we can keep the last print statement. 

        # self.get_logger().info(f"Trial ended")
        self.gazebo_sim_ctrl.pause()

        self.trial_elapsed_time = elapsed
        self.trial_success = False
        if collided:
            self.trial_status = "collided"
        elif elapsed >= self.timeout:
            self.trial_status = "timeout"
        else:
            self.trial_status = "succeeded"
            self.trial_success = True
        
        self.get_logger().info(f"Navigation {self.trial_status} with time {elapsed:.4f} (s).")
    
    def trial_cleanup(self):
        # report metric, generate log, and shutdown GazeboSimController
        
        world_idx = self.get_parameter('world_idx').value
        out_file = self.get_parameter('out_file').value

        # calculate metric
        if world_idx >= 300:
            path_length = self.goal_coor[0] - self.init_coor[0]
        else:
            path_file_name = os.path.join(get_package_share_directory("jackal_helper"), "worlds/BARN/path_files", f"path_{world_idx}.npy")
            path_array = np.load(path_file_name)
            path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
            path_array = np.insert(path_array, 0, (self.init_coor[0], self.init_coor[1]), axis=0)
            path_array = np.insert(path_array, len(path_array), (self.goal_coor[0], self.goal_coor[1]), axis=0)
            path_length = 0
            for p1, p2 in zip(path_array[:-1], path_array[1:]):
                path_length += compute_distance(p1, p2)
        
        # Navigation metric: 1_success *  optimal_time / clip(actual_time, 2 * optimal_time, 8 * optimal_time)
        optimal_time = path_length / 2
        nav_metric = int(self.trial_success) * optimal_time / np.clip(self.trial_elapsed_time, 2 * optimal_time, 8 * optimal_time)
        self.get_logger().info(f"Navigation metric: {nav_metric:.4f}")
        
        # writing to file
        out_path = os.path.join(self._get_pkg_src_path(), 'res')
        os.makedirs(out_path, exist_ok=True)
        out_path = os.path.join(out_path, out_file) 
        
        collided = self.trial_status == "collided"
        timeout = self.trial_status == "timeout"

        # safe write
        with self.file_lock:
            with open(out_path, "a") as f:
                f.write(
                    f"{world_idx} {int(self.trial_success)} {int(collided)} {int(timeout)} {self.trial_elapsed_time:.4f} {nav_metric:.4f}\n"
                )
        
        # shutdown GazeboSimController node
        self.gazebo_sim_ctrl.destroy_node()
    
    def _declare_parameters(self):
        self.declare_parameter('world_idx', 0)
        self.declare_parameter('out_file', 'out.txt')
        self.declare_parameter('timeout', 100)

    def _setup_initial_goal_positions(self):
        world_idx = self.get_parameter('world_idx').value
        if world_idx < 300:  # static environment from 0-299
            INIT_POSITION = [-2.25, 3, 1.57]  # in world frame
            GOAL_POSITION = [0, 10]  # relative to the initial position
        elif world_idx < 360:  # Dynamic environment from 300-359
            INIT_POSITION = [11, 0, 3.14]  # in world frame
            GOAL_POSITION = [-20, 0]  # relative to the initial position
        else:
            raise ValueError("World index %d does not exist" %world_idx)

        self.init_yaw_angle = INIT_POSITION[2]
        self.init_coor = (INIT_POSITION[0], INIT_POSITION[1])
        self.goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])
      
    def _get_pkg_src_path(self):
        # hack to get ws/src/jackal_helper
        workspace_path = dirname(dirname(dirname(dirname(get_package_share_directory("jackal_helper")))))
        BARN_challenge_src_path = os.path.join(workspace_path, "src", "The-Barn-Challenge-Ros2")
        return BARN_challenge_src_path

def main():
    rclpy.init()
    node = BARNRunner()
    node.reset_jackal_to_start()
    node.run_trial()
    node.trial_cleanup()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
