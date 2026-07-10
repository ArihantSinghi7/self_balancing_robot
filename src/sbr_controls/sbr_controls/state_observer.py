# This node converts the data received from IMU (Unit Quaternions) into pitch and yaw angle. 

import rclpy
from rclpy.node import Node
from sbr_interfaces.msg import RobotState
from sensor_msgs.msg import Imu
import math


class StateObserver(Node):

    # Class constructor
    def __init__(self):
        super().__init__("state_observer")

        # Creating and initializing variables
        self.pitch_angle: float = 0.0
        self.yaw_angle: float = 0.0
        self.pitch_rate: float = 0.0
        self.yaw_rate: float = 0.0
        self.is_fallen: bool = False
        
        # Creating a subscriber to get IMU data
        self.imu_data_subscriber = self.create_subscription(Imu,
                                                            "/imu/data",
                                                            self.callback_imu_data,
                                                            10)

        # Creating a publisher for robot's state
        self.robot_state_publisher = self.create_publisher(RobotState,
                                                           "robot_state",
                                                           10)
        
        # Creating a logger
        self.get_logger().info("'state_observer' node has been started.")

    
    # Callback method to get the IMU data
    def callback_imu_data(self, imu_data: Imu):
        
        # Unit Quaternions
        q_0 = imu_data.orientation.w
        q_1 = imu_data.orientation.x
        q_2 = imu_data.orientation.y
        q_3 = imu_data.orientation.z
        
        self.pitch_rate = imu_data.angular_velocity.y
        self.yaw_rate = imu_data.angular_velocity.z

        # Calculating pitch angle from the unit quaternions
        # Pitching means rotation about the y-axis
        # sin_y = 2(q0q2 - q1q3) 
        sin_p = 2*((q_0*q_2) - (q_1*q_3))
        sin_p = max(-1.0, min(1.0, sin_p))          # Clamp
        self.pitch_angle = math.asin(sin_p)
        
        # Calculating yaw angle from the unit quaternions
        # Yaw means rotation about the z-axis
        # tan_z = (sin_z*cos_y)/(cos_z*cos_y) 
        # sin_z*cos_y = 2(q0q3 + q1q2)
        # cos_z*cos_y = (q0^2 + q1^2 - q2^2 - q3^2)
        cz_cy = (q_0*q_0) + (q_1*q_1) - (q_2*q_2) - (q_3*q_3)
        sz_cy = 2*((q_0*q_3) + (q_1*q_2))
        self.yaw_angle = math.atan2(sz_cy, cz_cy)

        # Determine whether the robot has fallen
        # Assumption: If the robot is tilited more the 45 deg(pitch) it is considered as fallen.
        if abs(self.pitch_angle) >= 0.785:      
            self.is_fallen = True
        else:
            self.is_fallen = False

        # Publish the robot's state
        self.publish_robot_state()


    # Method to publish robot's state
    def publish_robot_state(self):

        msg = RobotState()
        msg.pitch = self.pitch_angle
        msg.yaw = self.yaw_angle
        msg.pitch_rate = self.pitch_rate
        msg.yaw_rate = self.yaw_rate
        msg.is_fallen = self.is_fallen

        self.robot_state_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = StateObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()