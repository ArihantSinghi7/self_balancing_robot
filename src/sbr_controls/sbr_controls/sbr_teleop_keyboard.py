# This node reads keyboard input and publishes velocity commands for teleoperation of the self balancing robot.

import rclpy
from rclpy.node import Node 
import sys
import tty
import termios
import threading
import time
from geometry_msgs.msg import TwistStamped

# NOTE:
# We need 'TwistStamped' because every few sec(as specified in the yaml file) the velocity command expires.
# So to know when the velocity expires we need a time stamp of the command.
# We need a 'frame_id' to let ros2_control know the reference frame in which the velocity is being expressed.

# Defining key bindings to control the robot
KEY_BINDINGS = {
    'W': (1.0, 0.0),   # FORWARD
    'S': (-1.0, 0.0),  # BACKWARD
    'A': (0.0, 1.0),   # COUNTER-CLOCKWISE ROTATION 
    'D': (0.0, -1.0),  # CLOCKWISE ROTATION
}

# Banner to show on the terminal to the user
BANNER = r"""
Self Balancing Robot Teleop
------------------
    W               
    |               W : Forward
A --|-- D           S : Backward
    |               A : Turn Left / Rotate Counter-Clockwise 
    S               D : Turn Right / Rotate Clockwise  

- Hold the keys for motion. Robot stops when no keys is pressed.

Ctrl+C: Quit
------------------
"""


class TeleopKeyboard(Node):

    # Class constructor
    def __init__(self):
        super().__init__("teleop_keyboard")

        # Declaring parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('vel_publishing_freq', 20)
        self.declare_parameter('key_press_check_freq', 20)
        self.declare_parameter('key_timeout', 0.1)

        # Getting parameters
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.vel_publishing_freq = self.get_parameter("vel_publishing_freq").value
        self.key_press_check_freq = self.get_parameter("key_press_check_freq").value
        self.key_timeout = self.get_parameter("key_timeout").value

        # Creating and initiating variables
        self.v_x: float = 0.0                              
        self.w_z: float = 0.0
        self.last_key_time: float = time.time()

        # Creating a publisher for teleop commands
        self.teleop_cmd_publisher = self.create_publisher(TwistStamped,
                                                          "/teleop_cmd",
                                                          10)
        
        # Creating a timer to get keyboard input
        self.get_keyboard_input_loop = self.create_timer((1.0/self.vel_publishing_freq), 
                                                         self.publish_teleop_cmd)

        # Creating a timer to check key presses
        self.watchdog_timer = self.create_timer((1.0/self.key_press_check_freq),
                                                self.watchdog)
        
        # NOTE:
        # Since we are running two threads and both either uses or updates  these variables: self.v_x, self.w_z
        # we need to create a thread lock such that only one thread can access these variables at a time.
        self.lock = threading.Lock()

        # NOTE:
        # We need a separate thread to read the input from keyboard. 
        # We don't want to stop publishing velocity while we wait for a keyboard input.

        # Keyboard thread
        self.kb_thread_running = True
        self.kb_thread = threading.Thread(target=self.get_keyboard_input, daemon=True)
					 # target=self.keyboard_loop: The function to run on a separate thread 
                     # daemon=True: Allows python to quit when the main thread finishes and not wait for this thread to finish
        self.kb_thread.start()			# Starts the new thread which runs parallel to the main thread

        print(BANNER)

        self.get_logger().info("'teleop_keyboard' node has been started.")


    # NOTE:
    # Normally when we use terminal, we write the entire command and press enter for the command to get registered.
    # But, for teleop operation we want to register the command as soon as it is entered. For this we use 'tty' to switch the terminal to raw mode.
    # After exiting the node we need to switch the terminal back to its original setting, for this we use 'termios' to save the old terminal setting and then revert back to it. 

    # Method to get the pressed key
    def get_key(self):
        fd = sys.stdin.fileno()                             # Gets the file descriptor for keyboard input.
        old = termios.tcgetattr(fd)                         # Gets the current terminal setting before we switch the terminal to raw mode.
        try:
            tty.setraw(fd)                                  # Sets the terminal to raw mode.
            key = sys.stdin.read(1).upper()                 # Reads exactly one character from the keyboard.
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)   # Resets the terminal back to its original state. 
                                                            # termios.TCSADRAIN: wait until all pending output has been written, then apply the terminal setting.
        return key


    # Method to check whether a key press has expired
    def watchdog(self):
        if time.time() - self.last_key_time > self.key_timeout:
            with self.lock:
                self.v_x = 0.0
                self.w_z = 0.0


    # Method to get keyboard input
    def get_keyboard_input(self):

        while self.kb_thread_running:
            key = self.get_key()              # This statement will blocks this thread until a key is pressed.
            self.last_key_time = time.time()  # Record time of every key press

            # Thread lock once the input is received
            with self.lock:

                if key in KEY_BINDINGS:     # Move Mobile Base
                    dx, dw = KEY_BINDINGS[key]
                    self.v_x = dx*self.linear_speed
                    self.w_z = dw*self.angular_speed

                elif key == '\x03':         # Ctrl+C
                    self.kb_thread_running = False
                    rclpy.try_shutdown()

                else: 
                    pass


    # Method to publish teleop commands
    def publish_teleop_cmd(self):
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()      # Gets time and converts into valid format  

        with self.lock:
            msg.twist.linear.x = self.v_x
            msg.twist.angular.z = self.w_z

        self.teleop_cmd_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.kb_thread_running = False		# Safety net to ensure that the keyboard thread is always stopped.
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()