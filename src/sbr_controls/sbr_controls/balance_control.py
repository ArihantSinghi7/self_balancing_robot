
import rclpy
from rclpy.node import Node
from sbr_interfaces.msg import RobotState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sbr_controls.clamp import Clamp
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class BalanceControl(Node):

    # Class constructor
    def __init__(self):
        super().__init__("balance_node")

        # Creating parameters
        self.declare_parameter("balance_loop_freq", 100) # Hz
        self.declare_parameter("velocity_loop_freq", 10)  # Hz

        # Declaring parameters
        self.balance_loop_freq = self.get_parameter("balance_loop_freq").value
        self.velocity_loop_freq = self.get_parameter("velocity_loop_freq").value
        
        # Creating and initiating variables
        # PID loop gains and variables
        self.previous_time = self.get_clock().now()
        self.integral = 0.0
        self.integral_max = 1.0
        self.p_gain: float = 30.0
        self.i_gain: float = 0.015
        self.d_gain: float = 0.5

        # Robot's state 
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.is_fallen = False
        self.current_velocity: float = 0.0

        # Desired state
        self.target_pitch_angle: float = 0.0
        self.corrected_target_pitch_angle = 0.0
        self.desired_velocity: float = 0.0
        
        # Velocity loop variables 
        self.v_gain: float = 0.1 

        # Limits:
        self.velocity_limits = Clamp(-8.0, 8.0) 
        self.vel_pitch_correction_limits = Clamp(-0.15, 0.15)   # Radians

        # QOS profile of '/diff_drive_controller/odom' topic publisher
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Creating a subscriber to get robot's state
        self.robot_state_subscriber = self.create_subscription(RobotState,
                                                               "robot_state",
                                                               self.callback_robot_state,
                                                               10)
        
        # Creating a subscriber to get robot's current velocty
        self.robot_velocity_subscriber = self.create_subscription(Odometry,
                                                                  "/diff_drive_controller/odom",
                                                                  self.callback_robot_velocity,
                                                                  odom_qos)

        # Creating a publisher for robot's velocity
        self.velocity_publisher = self.create_publisher(TwistStamped,
                                                        "/diff_drive_controller/cmd_vel",
                                                        10)

        # Creating a timer to run the position control loop
        self.pos_control_loop_timer = self.create_timer((1/self.balance_loop_freq), 
                                                    self.balance_control_loop)

        # Creating a timer to run the velocity control loop 
        self.vel_control_loop_timer = self.create_timer((1/self.velocity_loop_freq),
                                                self.vel_control_loop)
        
        # Creating a logger
        self.get_logger().info("'balance_control' node has been started.")


    # Callback method to get the robot's state
    def callback_robot_state(self, robot_state: RobotState):
        
        self.pitch_angle = robot_state.pitch
        self.yaw_angle = robot_state.yaw
        self.pitch_rate = robot_state.pitch_rate
        self.yaw_rate = robot_state.yaw_rate
        self.is_fallen = robot_state.is_fallen


    # Callback method to get robot's current velocity
    def callback_robot_velocity(self, robot_odom: Odometry):

        self.current_velocity = robot_odom.twist.twist.linear.x

      
    # Velocity control loop which keeps the velocity from drifting too much from the desired value
    def vel_control_loop(self):
        
        velocity_error = self.desired_velocity - self.current_velocity
        vel_pitch_correction = self.vel_pitch_correction_limits.clamp(self.v_gain * velocity_error)

        # Updates the target pitch angle based on the robot's current velocity and desired velocity 
        self.corrected_target_pitch_angle = self.target_pitch_angle + vel_pitch_correction


    # Control loop which keeps the robot upright and balanced
    def balance_control_loop(self):

        # Get time differential 'dt'
        now = self.get_clock().now()
        dt = (now - self.previous_time).nanoseconds / 1e9
        self.previous_time = now

        # Angular position error
        error = self.corrected_target_pitch_angle - self.pitch_angle
        
        # Initialize terms
        p_term = 0.0
        i_term = 0.0
        d_term = 0.0

        # If the robot falls stop it and then stop the control loops
        if self.is_fallen:
            self.integral = 0.0
            output_vel = 0.0                            
            self.pos_control_loop_timer.cancel()    
            self.vel_control_loop_timer.cancel()

        else:
            deadband = 0.005  # rad (~0.3°)
            if abs(error) > deadband:
                self.integral += error * dt
            
            self.integral = max(-self.integral_max, min(self.integral_max, self.integral)) # anti-windup
            
            p_term = -self.p_gain*error

            i_term = self.i_gain*self.integral

            d_term = self.d_gain*self.pitch_rate

            output_vel = self.velocity_limits.clamp(p_term + i_term + d_term)

        # Publish the robot's velocity
        self.publish_robot_velocity(float(output_vel)) 


    # Method to publish robot's velocity
    def publish_robot_velocity(self, output_vel: float):
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = output_vel
        msg.twist.angular.z = 0.0
        
        self.velocity_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()