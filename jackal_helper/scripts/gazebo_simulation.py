#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ros_gz_interfaces.srv import SetEntityPose, ControlWorld
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

import numpy as np
import time

# comes from robot.yaml file. JACKAL_ENTITY_NAME = "robot" if namespace == "" or "/"
# if namespace is something else, many things will need to change as code is hardcoded to fit namespace = ""
JACKAL_ENTITY_NAME = "robot"
DEBUG = False


def create_model_state(x:float, y:float, z:float, angle:float) -> [Entity, Pose]:
    # create Entity & Pose objects
    model_entity = Entity()
    model_entity.name = JACKAL_ENTITY_NAME

    model_pose = Pose()
    model_pose.position.x = float(x)
    model_pose.position.y = float(y)
    model_pose.position.z = float(z)
    # model_pose.orientation =  Quaternion()
    model_pose.orientation.x = 0.0
    model_pose.orientation.y = 0.0
    model_pose.orientation.z = float(np.sin(angle/2.))
    model_pose.orientation.w = float(np.cos(angle/2.))

    return (model_entity, model_pose)

class GazeboSimulationController(Node):
    def __init__(self, init_position = [0.0, 0.0, 0.0]):
        super().__init__('Gazebo_Simulation_Controller')

        self._initialize_service_clients()

        self._initialize_subscribers()

        # initialize model state: (Entity, Pose) 
        self._init_model_state = create_model_state(init_position[0], init_position[1], 0.1, init_position[2])

    def _initialize_subscribers(self):
        # initialize collision counter and relevant code
        self.collision_count = 0
        self.create_subscription(Bool, '/robot/touched', self._collision_cb, 10)

        # initialize subscriber for laser_scan
        self.create_subscription(LaserScan, '/front/scan', self._scan_cb, 10)
        self.latest_scan = None

        self.create_subscription(TFMessage, '/model/robot/pose', self._tf_msg_cb, 10)
        self.latest_pose = Pose()

    def _initialize_service_clients(self):
        self.set_entity_state_client = self.create_client(
            SetEntityPose,
            '/world/default/set_pose'
        )

        self.control_world_client = self.create_client(
            ControlWorld,
            '/world/default/control'
        )

    def _collision_cb(self, msg):
        if DEBUG:
            self.get_logger().info(f"/robot/collision callback received!")
            self.get_logger().info(f"msg.data: {msg.data}.")
            self.get_logger().info(f"collision_count: {self.collision_count}.")
        if msg.data:
            self.collision_count += 1 

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _tf_msg_cb(self, msg):
        for tf_stamped in msg.transforms:
            if tf_stamped.child_frame_id == JACKAL_ENTITY_NAME:
                # copying values without copying Vector3 object to keep the client side code same
                self.latest_pose.position.x = tf_stamped.transform.translation.x
                self.latest_pose.position.y = tf_stamped.transform.translation.y
                self.latest_pose.position.z = tf_stamped.transform.translation.z
                self.latest_pose.orientation = tf_stamped.transform.rotation

    def get_hard_collision(self):
        # hard collision count since last call
        collided = self.collision_count > 0
        self.collision_count = 0
        return collided

    def get_laser_scan(self):
        while self.latest_scan is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_scan

    def pause(self):
        req = ControlWorld.Request()
        req.world_control.pause = True
        while not self.control_world_client.wait_for_service(timeout_sec=1.0):
            if DEBUG:
                self.get_logger().info('Waiting for ControlWorld service...')
        future = self.control_world_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future
    
    def unpause(self):
        req = ControlWorld.Request()
        req.world_control.pause = False
        while not self.control_world_client.wait_for_service(timeout_sec=1.0):
            if DEBUG:
                self.get_logger().debug('Waiting for ControlWorld service...')
        future = self.control_world_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future

    def reset_model(self):
        """
        This resets the model. In ROS1 BARN, this was reset() which is ill-named as reset() means resetting the world and not just the model's pose. 
        """
        req = SetEntityPose.Request()
        req.entity = self._init_model_state[0]
        req.pose = self._init_model_state[1]
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            if DEBUG:
                self.get_logger().info('Waiting for set_pose service...')
        future = self.set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future

    def get_model_state(self):
        return self.latest_pose
    
    def reset_init_model_state(self, init_position = [0.0, 0.0, 0.0]):
        """
        Overwrite the initial model state
        Args:
            init_position (list, optional): initial model state in x, y, yaw. Defaults to [0, 0, 0].
        """
        self._init_model_state = create_model_state(init_position[0],init_position[1],0.1,init_position[2])

# TODO: 1. fix ros-gz bridge so that topics are standardized
# TODO: 2. wait for results of services before proceeding to next test.
def test():
    rclpy.init()

    node = GazeboSimulation()
    node.get_logger().info("object created.")
    rclpy.spin_once(node, timeout_sec=1.0)

    init_model_state = [10.0, -4.0, 0]
    node.get_logger().info(f"[1/8]: Reseting init model state to {init_model_state}...")
    node.reset_init_model_state(init_model_state)
    node.get_logger().info("[1/8]: Init model state reset.")
    rclpy.spin_once(node, timeout_sec=1.0)


    node.get_logger().info(f"[2/8]: Resetting model to {init_model_state}...")
    pose = node.reset_model()
    node.get_logger().info(f"[2/8]: Model reset.")
    rclpy.spin_once(node, timeout_sec=1.0)


    node.get_logger().info(f"[3/8]: Requesting pose...")
    pose = node.get_model_state().position
    node.get_logger().info(f"[3/8]: Model at({pose.x}, {pose.y}, {pose.z}).")
    rclpy.spin_once(node, timeout_sec=1.0)

    
    init_model_state = [-2.0, 3.0, 1.57]
    node.get_logger().info(f"[4/8]: Reseting init model state to {init_model_state}...")
    node.reset_init_model_state(init_model_state)
    node.get_logger().info("[4/8]: Init model state reset.")
    rclpy.spin_once(node, timeout_sec=1.0)


    node.get_logger().info(f"[5/8]: Resetting model to {init_model_state}...")
    pose = node.reset_model()
    node.get_logger().info(f"[5/8]: Model reset.")
    rclpy.spin_once(node, timeout_sec=1.0)


    node.get_logger().info(f"[6/8]: Requesting pose...")
    pose = node.get_model_state().position
    node.get_logger().info(f"[6/8]: Model at({pose.x}, {pose.y}, {pose.z}).")
    rclpy.spin_once(node, timeout_sec=1.0)


    node.get_logger().info("[7/8]: pausing physics...")
    node.pause()
    node.get_logger().info("[7/8]: paused.")
    rclpy.spin_once(node, timeout_sec=1.0)

    node.get_logger().info("[8/8]: unpausing physics...")
    node.unpause()
    node.get_logger().info("[8/8]: unpaused.")
    rclpy.spin_once(node, timeout_sec=1.0)

    node.get_logger().info("test complete. unpaused physics, node running for collision checking")

    rclpy.spin(node) 


    node.destroy_node()
    rclpy.shutdown()

def main():
    rclpy.init()
    node = GazeboSimulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
