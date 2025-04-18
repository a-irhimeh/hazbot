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
            self.get_logger().info('Successfully connected to Dobot M1')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Dobot M1: {str(e)}')
            print(f'\033[91mERROR - Connection Failed: {str(e)}\033[0m')  # Red error text
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
        self.base_linear_speed = 10.0  # base movement in mm per command
        self.base_angular_speed = 2.0  # base rotation in degrees per command
        self.base_z_speed = 5.0  # base Z movement in mm per command
        
        # Speed multiplier now controls coordinate skipping
        self.speed_multiplier = 1  # Start with no skipping (1 means use every coordinate)
        self.min_speed_multiplier = 1  # Minimum is no skipping
        self.max_speed_multiplier = 5  # Maximum will skip 4 out of 5 coordinates
        
        # Movement accumulation for coordinate skipping
        self.accumulated_x = 0.0
        self.accumulated_y = 0.0
        self.accumulated_z = 0.0
        self.accumulated_r = 0.0
        self.move_counter = 0  # Counter for coordinate skipping
        
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
        self.left_arm_orientation = True  # True for left, False for right
        self.motors_enabled = True  # Motors start enabled
        
        # Button debouncing
        self.last_speed_change_time = self.get_clock().now()
        self.last_orientation_change_time = self.get_clock().now()
        self.last_motor_toggle_time = self.get_clock().now()
        self.debounce_duration = 0.3  # seconds
        
    def update_speeds(self):
        """Update speed multiplier (coordinate skipping factor)"""
        self.get_logger().info(f'Speed level set to: {self.speed_multiplier}x (skipping {self.speed_multiplier - 1} coordinates)')
        
    def should_move(self):
        """Determine if we should move based on coordinate skipping"""
        self.move_counter = (self.move_counter + 1) % self.speed_multiplier
        return self.move_counter == 0
        
    def joy_callback(self, msg):
        try:
            current_time = self.get_clock().now()
            
            # Debug print for button presses and axis movements
            for i, button in enumerate(msg.buttons):
                if button:
                    print(f'\033[95mButton {i} pressed\033[0m')  # Magenta text for button presses
            
            for i, axis in enumerate(msg.axes):
                if abs(axis) > 0.5:  # Only show significant axis movement
                    print(f'\033[96mAxis {i} value: {axis:.2f}\033[0m')  # Cyan text for axis movement
            
            # Motor toggle using Start button (index 3)
            if len(msg.buttons) > 3:
                if msg.buttons[3] and (current_time - self.last_motor_toggle_time).nanoseconds / 1e9 > self.debounce_duration:
                    self.motors_enabled = not self.motors_enabled
                    try:
                        if self.motors_enabled:
                            # Re-enable motors
                            self.device._set_ptp_joint_params(100, 100, 100, 100, 100, 100, 100, 100)
                            self.device._set_ptp_coordinate_params(velocity=200, acceleration=200)
                            self.device._set_ptp_common_params(velocity=100, acceleration=100)
                            self.get_logger().info('Motors enabled')
                            print('\033[92mMotors enabled successfully\033[0m')
                        else:
                            # Disable motors by setting speeds to 0
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
                                self.speed_multiplier = min(self.max_speed_multiplier, self.speed_multiplier + 1)
                                self.update_speeds()
                                self.last_speed_change_time = current_time
                        elif msg.axes[7] < 0:  # D-pad down
                            if (current_time - self.last_speed_change_time).nanoseconds / 1e9 > self.debounce_duration:
                                self.speed_multiplier = max(self.min_speed_multiplier, self.speed_multiplier - 1)
                                self.update_speeds()
                                self.last_speed_change_time = current_time
                    
                    # Arm orientation control (R2 button - index 9)
                    if len(msg.buttons) > 9:
                        if msg.buttons[9] and (current_time - self.last_orientation_change_time).nanoseconds / 1e9 > self.debounce_duration:
                            try:
                                self.left_arm_orientation = not self.left_arm_orientation
                                self.device._set_arm_orientation('L' if self.left_arm_orientation else 'R')
                                orientation_str = "Left" if self.left_arm_orientation else "Right"
                                self.get_logger().info(f'Arm orientation set to: {orientation_str}')
                                print(f'\033[94mArm orientation changed to: {orientation_str}\033[0m')
                            except Exception as e:
                                self.get_logger().error(f'Error changing arm orientation: {str(e)}')
                                print(f'\033[91mERROR - Failed to change arm orientation: {str(e)}\033[0m')
                                print(f'Traceback:\n{"".join(traceback.format_tb(e.__traceback__))}')
                            self.last_orientation_change_time = current_time
                    
                    # Calculate movement amounts
                    x_move = msg.axes[2] * self.base_linear_speed
                    y_move = msg.axes[3] * self.base_linear_speed
                    
                    z_move = 0.0
                    if len(msg.buttons) > 11:
                        if msg.buttons[10]:  # L1 for down
                            z_move = -self.base_z_speed
                        elif msg.buttons[11]:  # R1 for up
                            z_move = self.base_z_speed
                    
                    # Rotation using D-pad left/right (axes 6)
                    r_move = 0.0
                    if len(msg.axes) >= 7:
                        if msg.axes[6] > 0:  # D-pad left
                            r_move = -self.base_angular_speed
                            print('\033[96mRotating counter-clockwise\033[0m')
                        elif msg.axes[6] < 0:  # D-pad right
                            r_move = self.base_angular_speed
                            print('\033[96mRotating clockwise\033[0m')
                    
                    # Accumulate movements
                    self.accumulated_x += x_move
                    self.accumulated_y += y_move
                    self.accumulated_z += z_move
                    self.accumulated_r += r_move
                    
                    # Only move if we should (based on coordinate skipping)
                    if self.should_move():
                        try:
                            # Calculate new position using accumulated movements
                            new_x = self.current_x + self.accumulated_x
                            new_y = self.current_y + self.accumulated_y
                            new_z = self.current_z + self.accumulated_z
                            new_r = self.current_r + self.accumulated_r
                            
                            # Move the arm
                            self.device.move_to(new_x, new_y, new_z, new_r, wait=True)
                            
                            # Update current position
                            self.current_x, self.current_y, self.current_z, self.current_r, _, _, _, _ = self.device.pose()
                            
                            # Reset accumulations
                            self.accumulated_x = 0.0
                            self.accumulated_y = 0.0
                            self.accumulated_z = 0.0
                            self.accumulated_r = 0.0
                        except Exception as e:
                            self.get_logger().error(f'Error during movement: {str(e)}')
                            print(f'\033[91mERROR - Movement failed: {str(e)}\033[0m')
                            print(f'Attempted move to: X={new_x:.2f}, Y={new_y:.2f}, Z={new_z:.2f}, R={new_r:.2f}')
                            print(f'Traceback:\n{"".join(traceback.format_tb(e.__traceback__))}')
                    
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