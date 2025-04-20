#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from serial.tools import list_ports
from pydobot import Dobot
import math
import traceback
import time
from threading import Lock

class DobotM1JoystickControl(Node):
    def __init__(self):
        super().__init__('dobot_m1_joystick_control')
        
        # Initialize Dobot M1
        try:
            port = list_ports.comports()[0].device
            self.device = Dobot(port=port, verbose=False)
            self.device._Clear_All_Alarm_States()
            self.device._set_arm_orientation('L')  # Set to left-handed orientation
            # Set higher speeds for more responsive movement
            self.device._set_ptp_joint_params(400, 400, 400, 400, 400, 400, 400, 400)
            self.device._set_ptp_coordinate_params(velocity=400, acceleration=400)
            self.device._set_ptp_common_params(velocity=200, acceleration=200)
            self.get_logger().info('Successfully connected to Dobot M1')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Dobot M1: {str(e)}')
            print(f'\033[91mERROR - Connection Failed: {str(e)}\033[0m')
            return

        # Thread safety
        self.device_lock = Lock()
        
        # Get initial position
        try:
            with self.device_lock:
                self.current_x, self.current_y, self.current_z, self.current_r, _, _, _, _ = self.device.pose()
                self.target_x = self.current_x
                self.target_y = self.current_y
                self.target_z = self.current_z
                self.target_r = self.current_r
        except Exception as e:
            self.get_logger().error(f'Failed to get initial pose: {str(e)}')
            print(f'\033[91mERROR - Failed to get initial pose: {str(e)}\033[0m')
            return
        
        # Movement parameters
        self.base_linear_speed = 50.0  # mm per command
        self.base_angular_speed = 10.0  # degrees per command
        self.base_z_speed = 25.0  # mm per command
        
        # Control modes
        self.CONTROL_MODE_XYZ = 0
        self.CONTROL_MODE_JOINT = 1
        self.control_mode = self.CONTROL_MODE_XYZ
        
        # Button mappings (PS4 controller)
        self.BTN_X = 2        # Gripper
        self.BTN_CIRCLE = 1   # Suction
        self.BTN_TRIANGLE = 3 # Change control mode
        self.BTN_SQUARE = 0   # Change orientation
        self.BTN_L1 = 4       # Z down
        self.BTN_R1 = 5       # Z up
        self.BTN_L2 = 6       # Decrease speed
        self.BTN_R2 = 7       # Increase speed
        self.BTN_SHARE = 8    # Home position
        self.BTN_OPTIONS = 9  # Emergency stop
        
        # Axis mappings
        self.AXIS_LX = 0  # X movement in XYZ mode / J1 in Joint mode
        self.AXIS_LY = 1  # Y movement in XYZ mode / J2 in Joint mode
        self.AXIS_RX = 3  # Rotation in XYZ mode / J3 in Joint mode
        self.AXIS_RY = 4  # Z movement in XYZ mode / J4 in Joint mode
        
        # Movement thresholds and timing
        self.min_move_interval = 0.02  # 20ms for better responsiveness
        self.last_move_time = time.time()
        self.last_position_check = time.time()
        self.position_check_interval = 0.5  # 500ms position check interval
        self.joy_deadzone = 0.15  # Ignore small joystick movements
        
        # Speed control
        self.speed_multiplier = 1.0
        self.min_speed_multiplier = 0.2
        self.max_speed_multiplier = 2.0
        self.speed_increment = 0.1
        
        # Effective speeds
        self.linear_speed = self.base_linear_speed * self.speed_multiplier
        self.angular_speed = self.base_angular_speed * self.speed_multiplier
        self.z_speed = self.base_z_speed * self.speed_multiplier
        
        # Button debouncing
        self.last_button_press = {
            'mode': 0,
            'orientation': 0,
            'gripper': 0,
            'suction': 0,
            'home': 0,
            'estop': 0,
            'speed_up': 0,
            'speed_down': 0
        }
        self.button_debounce_time = 0.3
        
        # Create subscription to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            1
        )
        
        # Initialize states
        self.gripper_state = False
        self.suction_state = False
        self.left_arm_orientation = True
        self.motors_enabled = True
        self.is_estopped = False
        
        # Error recovery
        self.consecutive_errors = 0
        self.max_consecutive_errors = 3
        self.error_reset_interval = 5.0
        self.last_error_time = time.time()
        
        # Create watchdog timer
        self.create_timer(1.0, self.watchdog_callback)
        
    def watchdog_callback(self):
        """Periodic check to ensure robot is responsive"""
        try:
            if self.is_estopped:
                return

            with self.device_lock:
                # Clear alarms
                self.device._Clear_All_Alarm_States()
                
                # Check if we need to re-enable motors
                if not self.motors_enabled:
                    self.device._set_ptp_joint_params(400, 400, 400, 400, 400, 400, 400, 400)
                    self.device._set_ptp_coordinate_params(velocity=400, acceleration=400)
                    self.device._set_ptp_common_params(velocity=200, acceleration=200)
                    self.motors_enabled = True
                    self.get_logger().info('Motors re-enabled by watchdog')
                
                # Update current position
                try:
                    self.current_x, self.current_y, self.current_z, self.current_r, _, _, _, _ = self.device.pose()
                except Exception as e:
                    self.get_logger().warn(f'Failed to get pose in watchdog: {str(e)}')
                
        except Exception as e:
            self.get_logger().error(f'Watchdog error: {str(e)}')
            # Don't increment error counter here to avoid cascading failures

    def update_speeds(self):
        """Update all speed values based on the current multiplier"""
        self.linear_speed = self.base_linear_speed * self.speed_multiplier
        self.angular_speed = self.base_angular_speed * self.speed_multiplier
        self.z_speed = self.base_z_speed * self.speed_multiplier
        self.get_logger().info(f'Speed multiplier: {self.speed_multiplier:.1f}x')
    
    def handle_speed_change(self, increase: bool, current_time: float):
        """Handle speed multiplier changes"""
        if current_time - self.last_button_press['speed_up' if increase else 'speed_down'] > self.button_debounce_time:
            if increase:
                self.speed_multiplier = min(self.max_speed_multiplier, self.speed_multiplier + self.speed_increment)
            else:
                self.speed_multiplier = max(self.min_speed_multiplier, self.speed_multiplier - self.speed_increment)
            self.update_speeds()
            self.last_button_press['speed_up' if increase else 'speed_down'] = current_time
    
    def handle_home_position(self, current_time: float):
        """Move to home position"""
        if current_time - self.last_button_press['home'] > self.button_debounce_time:
            try:
                with self.device_lock:
                    self.device.move_to(200, 0, 0, 0, wait=False)  # Default home position
                self.get_logger().info('Moving to home position')
                self.last_button_press['home'] = current_time
            except Exception as e:
                self.get_logger().error(f'Failed to move to home: {str(e)}')
    
    def joy_callback(self, msg):
        if not hasattr(self, 'device'):
            self.get_logger().error('No device connection available')
            return
            
        if self.is_estopped:
            return
            
        current_time = time.time()
        
        # Debug output for joystick state
        if any(abs(axis) > self.joy_deadzone for axis in msg.axes) or any(msg.buttons):
            self.get_logger().info(f'Axes: {[f"{x:.2f}" for x in msg.axes]}')
            self.get_logger().info(f'Buttons: {msg.buttons}')
            # Add detailed axis diagnostics
            if len(msg.axes) > self.AXIS_RY:
                self.get_logger().info(f'Detailed axes:'
                                     f'\n  Left X (AXIS_LX={self.AXIS_LX}): {msg.axes[self.AXIS_LX]:.3f}'
                                     f'\n  Left Y (AXIS_LY={self.AXIS_LY}): {msg.axes[self.AXIS_LY]:.3f}'
                                     f'\n  Right X (AXIS_RX={self.AXIS_RX}): {msg.axes[self.AXIS_RX]:.3f}'
                                     f'\n  Right Y (AXIS_RY={self.AXIS_RY}): {msg.axes[self.AXIS_RY]:.3f}')
            
            # Check for potentially stuck axes
            stuck_axes = [i for i, val in enumerate(msg.axes) if abs(abs(val) - 1.0) < 0.01]
            if stuck_axes:
                self.get_logger().warn(f'Possible stuck axes detected: {stuck_axes}')
        
        # Handle E-stop
        if len(msg.buttons) > self.BTN_OPTIONS and msg.buttons[self.BTN_OPTIONS]:
            if current_time - self.last_button_press['estop'] > self.button_debounce_time:
                self.is_estopped = True
                try:
                    with self.device_lock:
                        self.device._set_ptp_joint_params(0, 0, 0, 0, 0, 0, 0, 0)
                        self.motors_enabled = False
                    self.get_logger().warn('Emergency stop activated')
                except Exception as e:
                    self.get_logger().error(f'Failed to E-stop: {str(e)}')
                self.last_button_press['estop'] = current_time
                return
        
        # Rate limiting
        if current_time - self.last_move_time < self.min_move_interval:
            return
            
        # Reset error count if enough time has passed
        if current_time - self.last_error_time > self.error_reset_interval:
            self.consecutive_errors = 0
        
        try:
            # Handle mode change (Triangle button)
            if len(msg.buttons) > self.BTN_TRIANGLE and msg.buttons[self.BTN_TRIANGLE]:
                if current_time - self.last_button_press['mode'] > self.button_debounce_time:
                    self.control_mode = (self.control_mode + 1) % 2
                    self.get_logger().info(f'Control mode: {"XYZ" if self.control_mode == self.CONTROL_MODE_XYZ else "Joint"}')
                    self.last_button_press['mode'] = current_time
            
            # Handle orientation change (Square button)
            if len(msg.buttons) > self.BTN_SQUARE and msg.buttons[self.BTN_SQUARE]:
                if current_time - self.last_button_press['orientation'] > self.button_debounce_time:
                    with self.device_lock:
                        self.left_arm_orientation = not self.left_arm_orientation
                        self.device._set_arm_orientation('L' if self.left_arm_orientation else 'R')
                        self.get_logger().info(f'Arm orientation: {"Left" if self.left_arm_orientation else "Right"}')
                    self.last_button_press['orientation'] = current_time
            
            # Handle speed changes
            if len(msg.buttons) > self.BTN_R2 and msg.buttons[self.BTN_R2]:
                self.handle_speed_change(True, current_time)
            if len(msg.buttons) > self.BTN_L2 and msg.buttons[self.BTN_L2]:
                self.handle_speed_change(False, current_time)
            
            # Handle home position
            if len(msg.buttons) > self.BTN_SHARE and msg.buttons[self.BTN_SHARE]:
                self.handle_home_position(current_time)
            
            # Calculate movements based on control mode
            if self.control_mode == self.CONTROL_MODE_XYZ:
                # XYZ mode movement
                x_move = msg.axes[self.AXIS_LX] * self.linear_speed if abs(msg.axes[self.AXIS_LX]) > self.joy_deadzone else 0
                y_move = -msg.axes[self.AXIS_LY] * self.linear_speed if abs(msg.axes[self.AXIS_LY]) > self.joy_deadzone else 0
                
                # Z movement requires L1 (down) or R1 (up) buttons
                z_move = 0
                if len(msg.buttons) > self.BTN_R1:
                    if msg.buttons[self.BTN_R1]:  # Up
                        z_move = self.z_speed
                    elif msg.buttons[self.BTN_L1]:  # Down
                        z_move = -self.z_speed
                
                r_move = msg.axes[self.AXIS_RX] * self.angular_speed if abs(msg.axes[self.AXIS_RX]) > self.joy_deadzone else 0
                
                # Update target position
                self.target_x = self.current_x + x_move
                self.target_y = self.current_y + y_move
                self.target_z = self.current_z + z_move
                self.target_r = self.current_r + r_move
                
                # Send movement command if there's significant movement
                if abs(x_move) > 0.1 or abs(y_move) > 0.1 or abs(z_move) > 0.1 or abs(r_move) > 0.1:
                    with self.device_lock:
                        self.device.move_to(self.target_x, self.target_y, self.target_z, self.target_r, wait=False)
                    self.last_move_time = current_time
            
            else:  # CONTROL_MODE_JOINT
                # Joint mode movement (if supported by your Dobot implementation)
                j1 = msg.axes[self.AXIS_LX] * self.angular_speed if abs(msg.axes[self.AXIS_LX]) > self.joy_deadzone else 0
                j2 = -msg.axes[self.AXIS_LY] * self.angular_speed if abs(msg.axes[self.AXIS_LY]) > self.joy_deadzone else 0
                j3 = msg.axes[self.AXIS_RX] * self.angular_speed if abs(msg.axes[self.AXIS_RX]) > self.joy_deadzone else 0
                j4 = -msg.axes[self.AXIS_RY] * self.angular_speed if abs(msg.axes[self.AXIS_RY]) > self.joy_deadzone else 0
                
                if abs(j1) > 0.1 or abs(j2) > 0.1 or abs(j3) > 0.1 or abs(j4) > 0.1:
                    with self.device_lock:
                        # Note: Implement joint movement if your Dobot API supports it
                        pass
            
            # Handle gripper control (X button)
            if len(msg.buttons) > self.BTN_X and msg.buttons[self.BTN_X]:
                if current_time - self.last_button_press['gripper'] > self.button_debounce_time:
                    with self.device_lock:
                        self.gripper_state = not self.gripper_state
                        self.device.grip(self.gripper_state)
                        self.get_logger().info(f'Gripper: {"On" if self.gripper_state else "Off"}')
                    self.last_button_press['gripper'] = current_time
            
            # Handle suction control (Circle button)
            if len(msg.buttons) > self.BTN_CIRCLE and msg.buttons[self.BTN_CIRCLE]:
                if current_time - self.last_button_press['suction'] > self.button_debounce_time:
                    with self.device_lock:
                        self.suction_state = not self.suction_state
                        self.device.suck(self.suction_state)
                        self.get_logger().info(f'Suction: {"On" if self.suction_state else "Off"}')
                    self.last_button_press['suction'] = current_time
                    
        except Exception as e:
            self.get_logger().error(f'Movement error: {str(e)}')
            self.consecutive_errors += 1
            self.last_error_time = current_time
    
    def __del__(self):
        if hasattr(self, 'device'):
            try:
                with self.device_lock:
                    self.device.close()
            except:
                pass

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