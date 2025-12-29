import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        
        # Run at 50Hz (50 times per second)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info("Sine Wave Publisher Started! Watch the robot dance.")

    def timer_callback(self):
        current_time = time.time() - self.start_time
        
        # Calculate smooth sine wave motion
        # We move Shoulder Pan (Joint 0) and Elbow (Joint 2)
        joint0 = 1.0 * math.sin(current_time)        # Swings left/right
        joint1 = -1.57                               # Keep shoulder lift steady
        joint2 = 1.57 + (0.5 * math.sin(current_time)) # Bob elbow up/down
        
        msg = Float64MultiArray()
        # [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
        msg.data = [joint0, joint1, joint2, -1.57, -1.57, 0.0]
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()