import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import JointState 
import numpy as np
import mujoco
import mujoco.viewer
import time

# Use relative import for the sibling file
from .ur5_env import UR5ReachEnv

class MujocoBridge(Node):
    def __init__(self):
        super().__init__('mujoco_bridge')
        
        self.get_logger().info("Starting MuJoCo Simulation with Feedback...")
        self.env = UR5ReachEnv(render_mode=None)
        self.env.reset()
        
        # Launch Viewer
        self.viewer = mujoco.viewer.launch_passive(self.env.model, self.env.data)
        
        # 1. SUBSCRIBER (Listening for Commands)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.listener_callback,
            10)
        
        # 2. PUBLISHER (Speaking the State)
        self.state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # 3. Timer
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.target_pos = np.zeros(6)
        
        # Define Joint Names (Standard UR5e names)
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

    def listener_callback(self, msg):
        if len(msg.data) == 6:
            self.target_pos = np.array(msg.data, dtype=np.float32)

    def timer_callback(self):
        # --- CONTROL ---
        self.env.data.qpos[:6] = self.target_pos
        mujoco.mj_step(self.env.model, self.env.data)
        self.viewer.sync()
        
        # --- FEEDBACK (SENSORS) --- 
        # Create the message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Read actual positions from MuJoCo
        current_pos = self.env.data.qpos[:6].tolist()
        current_vel = self.env.data.qvel[:6].tolist()
        
        msg.position = current_pos
        msg.velocity = current_vel
        
        # Send it to ROS
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()