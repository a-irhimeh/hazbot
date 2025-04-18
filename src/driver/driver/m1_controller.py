#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from serial.tools import list_ports
from pydobot import Dobot
import math
import traceback

class DobotM1JoystickControl(Node):
    def __init__(self):
        super().__init__('dobot_m1_joystick_control')
        
        # Initialize Dobot M1
        try:
            port = list_ports.comports()[0].device
            self.device = Dobot(port=port, verbose=False)
            self.device._Clear_All_Alarm_States()
            self.device._set_arm_orientation('L')  # Set to left-handed orientation
            # Set initial high speeds
            self.device._set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
            self.device._set_ptp_coordinate_params(velocity=200, acceleration=200)
            self.device._set_ptp_common_params(velocity=100, acceleration=100)
            self.get_logger().info('Successfully connected to Dobot M1')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Dobot M1: {str(e)}')
            print(f'\033[91mERROR - Connection Failed: {str(e)}\033[0m')
            print(f'Traceback:\n{"".join(traceback.format_tb(e.__traceback__))}')
            return

        # Get initial position
        try:
            self.current_x, self.current_y, self.current_z, self.current_r, _, _, _, _ = self.device.pose()
        except Exception as e:
            self.get_logger().error(f'Failed to get initial pose: {str(e)}')
            print(f'\033[91mERROR - Failed to get initial pose: {str(e)}\033[0m')
            print(f'Traceback:\n{"".join(traceback.format_tb(e.__traceback__))}')
            return
        
        # Movement parameters
        self.base_linear_speed = 20.0  # Increased base speed in mm per command
        self.base_angular_speed = 5.0  # Increased base rotation in degrees per command
        self.base_z_speed = 10.0  # Increased base Z movement in mm per command
        
        # Speed multiplier for direct speed control
        self.speed_multiplier = 1.0
        self.min_speed_multiplier = 0.5
        self.max_speed_multiplier = 3.0
        
        # Effective speeds
        self.linear_speed = self.base_linear_speed * self.speed_multiplier
        self.angular_speed = self.base_angular_speed * self.speed_multiplier
        self.z_speed = self.base_z_speed * self.speed_multiplier
        
        # Create subscription to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Initialize states
        self.gripper_state = False
        self.suction_state = False
        self.left_arm_orientation = True
        self.motors_enabled = True
        
        # Button debouncing
        self.last_speed_change_time = self.get_clock().now()
        self.last_orientation_change_time = self.get_clock().now()
        self.last_motor_toggle_time = self.get_clock().now()
        self.debounce_duration = 0.3
        
    def update_speeds(self):
        """Update all speed values based on the current multiplier"""
        self.linear_speed = self.base_linear_speed * self.speed_multiplier
        self.angular_speed = self.base_angular_speed * self.speed_multiplier
        self.z_speed = self.base_z_speed * self.speed_multiplier
        self.get_logger().info(f'Speed multiplier set to: {self.speed_multiplier:.1f}x')
        
    def joy_callback(self, msg):
        try:
            current_time = self.get_clock().now()
            
            # Debug print for button presses and axis movements
            for i, button in enumerate(msg.buttons):
                if button:
                    print(f'\033[95mButton {i} pressed\033[0m')
            
            for i, axis in enumerate(msg.axes):
                if abs(axis) > 0.5:
                    print(f'\033[96mAxis {i} value: {axis:.2f}\033[0m')
            
            # Motor toggle using Start button (index 3)
            if len(msg.buttons) > 3:
                if msg.buttons[3] and (current_time - self.last_motor_toggle_time).nanoseconds / 1e9 > self.debounce_duration:
                    self.motors_enabled = not self.motors_enabled
                    try:
                        if self.motors_enabled:
                            # Re-enable motors with high speeds
                            self.device._set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
                            self.device._set_ptp_coordinate_params(velocity=200, acceleration=200)
                            self.device._set_ptp_common_params(velocity=100, acceleration=100)
                            self.get_logger().info('Motors enabled')
                            print('\033[92mMotors enabled successfully\033[0m')
                        else:
                            # Disable motors
                            self.device._set_ptp_joint_params(0, 0, 0, 0, 0, 0, 0, 0)
                            self.device._set_ptp_coordinate_params(velocity=0, acceleration=0)
                            self.device._set_ptp_common_params(velocity=0, acceleration=0)
                            self.get_logger().info('Motors disabled')
                            print('\033[93mMotors disabled\033[0m')
                    except Exception as e:
                        self.get_logger().error(f'Error toggling motors: {str(e)}')
                        print(f'\033[91mERROR - Failed to toggle motors: {str(e)}\033[0m')
                        print(f'Traceback:\n{"".join(traceback.format_tb(e.__traceback__))}')
                    self.last_motor_toggle_time = current_time
            
            # Only process movement commands if motors are enabled
            if self.motors_enabled:
                try:
                    # Speed control using D-pad up/down (axes 7)
                    if len(msg.axes) >= 8:
                        if msg.axes[7] > 0:  # D-pad up
                            if (current_time - self.last_speed_change_time).nanoseconds / 1e9 > self.debounce_duration:
                                self.speed_multiplier = min(self.max_speed_multiplier, self.speed_multiplier + 0.2)
                                self.update_speeds()
                                self.last_speed_change_time = current_time
                        elif msg.axes[7] < 0:  # D-pad down
                            if (current_time - self.last_speed_change_time).nanoseconds / 1e9 > self.debounce_duration:
                                self.speed_multiplier = max(self.min_speed_multiplier, self.speed_multiplier - 0.2)
                                self.update_speeds()
                                self.last_speed_change_time = current_time
                    
                    # Calculate movements with direct speed control
                    x_move = msg.axes[2] * self.linear_speed
                    y_move = msg.axes[3] * self.linear_speed
                    
                    z_move = 0.0
                    if len(msg.buttons) > 11:
                        if msg.buttons[10]:  # L1 for down
                            z_move = -self.z_speed
                        elif msg.buttons[11]:  # R1 for up
                            z_move = self.z_speed
                    
                    # Rotation using D-pad left/right (axes 6)
                    r_move = 0.0
                    if len(msg.axes) >= 7:
                        if msg.axes[6] > 0:  # D-pad left
                            r_move = -self.angular_speed
                            print('\033[96mRotating counter-clockwise\033[0m')
                        elif msg.axes[6] < 0:  # D-pad right
                            r_move = self.angular_speed
                            print('\033[96mRotating clockwise\033[0m')
                    
                    # Calculate new position
                    new_x = self.current_x + x_move
                    new_y = self.current_y + y_move
                    new_z = self.current_z + z_move
                    new_r = self.current_r + r_move
                    
                    # Move the arm (without wait for faster response)
                    self.device.move_to(new_x, new_y, new_z, new_r, wait=False)
                    
                    # Update current position
                    self.current_x = new_x
                    self.current_y = new_y
                    self.current_z = new_z
                    self.current_r = new_r
                    
                    # Handle gripper control (X button - index 0)
                    if len(msg.buttons) > 0 and msg.buttons[0]:
                        try:
                            self.gripper_state = not self.gripper_state
                            self.device.grip(self.gripper_state)
                            print(f'\033[94mGripper {"activated" if self.gripper_state else "deactivated"}\033[0m')
                        except Exception as e:
                            self.get_logger().error(f'Error controlling gripper: {str(e)}')
                            print(f'\033[91mERROR - Gripper control failed: {str(e)}\033[0m')
                            print(f'Traceback:\n{"".join(traceback.format_tb(e.__traceback__))}')
                    
                    # Handle suction control (Triangle button - index 3)
                    if len(msg.buttons) > 3 and msg.buttons[3]:
                        try:
                            self.suction_state = not self.suction_state
                            self.device.suck(self.suction_state)
                            print(f'\033[94mSuction {"activated" if self.suction_state else "deactivated"}\033[0m')
                        except Exception as e:
                            self.get_logger().error(f'Error controlling suction: {str(e)}')
                            print(f'\033[91mERROR - Suction control failed: {str(e)}\033[0m')
                            print(f'Traceback:\n{"".join(traceback.format_tb(e.__traceback__))}')
                
                except Exception as e:
                    self.get_logger().error(f'Error in movement processing: {str(e)}')
                    print(f'\033[91mERROR - Movement processing failed: {str(e)}\033[0m')
                    print(f'Traceback:\n{"".join(traceback.format_tb(e.__traceback__))}')
                    
        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {str(e)}')
            print(f'\033[91mERROR in joy_callback: {str(e)}\033[0m')
            print(f'Traceback:\n{"".join(traceback.format_tb(e.__traceback__))}')
    
    def __del__(self):
        if hasattr(self, 'device'):
            self.device.close()

def main(args=None):
    rclpy.init(args=args)
    node = DobotM1JoystickControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()