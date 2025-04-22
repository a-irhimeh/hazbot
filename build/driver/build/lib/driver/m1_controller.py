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
        
        # Button mappings (PS3 controller)
        self.BTN_SELECT = 0    # Reset position to center
        self.BTN_L3 = 1        # Left analog button (not used)
        self.BTN_R3 = 2        # Right analog button (not used)
        self.BTN_START = 3     # Enable/disable motors
        self.BTN_DPAD_UP = 4   # Increase speed
        self.BTN_DPAD_RIGHT = 5  # Not used
        self.BTN_DPAD_DOWN = 6   # Decrease speed
        self.BTN_DPAD_LEFT = 7   # Not used
        self.BTN_L2 = 8        # Not used
        self.BTN_R2 = 9        # Change orientation
        self.BTN_L1 = 10       # Z down
        self.BTN_R1 = 11       # Z up
        self.BTN_TRIANGLE = 12  # Suction options
        self.BTN_CIRCLE = 13   # Clear alarms
        self.BTN_X = 14        # Suction on/off
        self.BTN_SQUARE = 15   # Not used
        self.BTN_PS = 16       # Not used
        
        # Axis mappings for PS3 (these are correct)
        self.AXIS_LX = 0      # Not used
        self.AXIS_LY = 1      # Not used
        self.AXIS_RX = 2      # X movement in XYZ mode
        self.AXIS_RY = 3      # Y movement in XYZ mode
        
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
            
        current_time = time.time()
        
        # Handle motor enable/disable (Start button)
        if len(msg.buttons) > self.BTN_START and msg.buttons[self.BTN_START]:
            if current_time - self.last_button_press['estop'] > self.button_debounce_time:
                if self.is_estopped:
                    # Re-enable the robot
                    self.is_estopped = False
                    with self.device_lock:
                        self.device._set_ptp_joint_params(400, 400, 400, 400, 400, 400, 400, 400)
                        self.device._set_ptp_coordinate_params(velocity=400, acceleration=400)
                        self.device._set_ptp_common_params(velocity=200, acceleration=200)
                        self.motors_enabled = True
                    self.get_logger().info('Motors enabled')
                else:
                    # Disable motors
                    self.is_estopped = True
                    with self.device_lock:
                        self.device._set_ptp_joint_params(0, 0, 0, 0, 0, 0, 0, 0)
                        self.motors_enabled = False
                    self.get_logger().warn('Motors disabled')
                self.last_button_press['estop'] = current_time
                return
        
        if self.is_estopped:
            self.get_logger().warn('Robot is e-stopped. Press Start to re-enable.')
            return
            
        # Debug output for button presses
        for i, button in enumerate(msg.buttons):
            if button:
                self.get_logger().info(f'Button {i} pressed')
        
        # Debug output for non-zero axes
        for i, axis in enumerate(msg.axes):
            if abs(axis) > 0.1:
                self.get_logger().info(f'Axis {i} value: {axis:.2f}')
        
        # Rate limiting
        if current_time - self.last_move_time < self.min_move_interval:
            return
            
        try:
            # Handle clear alarms (Circle button)
            if len(msg.buttons) > self.BTN_CIRCLE and msg.buttons[self.BTN_CIRCLE]:
                if current_time - self.last_button_press['alarm'] > self.button_debounce_time:
                    with self.device_lock:
                        self.device._Clear_All_Alarm_States()
                    self.get_logger().info('Cleared all alarms')
                    self.last_button_press['alarm'] = current_time
            
            # Handle orientation change (R2 button)
            if len(msg.buttons) > self.BTN_R2 and msg.buttons[self.BTN_R2]:
                if current_time - self.last_button_press['orientation'] > self.button_debounce_time:
                    try:
                        with self.device_lock:
                            self.left_arm_orientation = not self.left_arm_orientation
                            orientation = 'L' if self.left_arm_orientation else 'R'
                            self.device._set_arm_orientation(orientation)
                            self.get_logger().info(f'Changed arm orientation to: {orientation}')
                        self.last_button_press['orientation'] = current_time
                    except Exception as e:
                        self.get_logger().error(f'Failed to change orientation: {str(e)}')
            
            # Handle speed changes (D-pad Up/Down)
            if len(msg.buttons) > self.BTN_DPAD_UP and msg.buttons[self.BTN_DPAD_UP]:
                self.handle_speed_change(True, current_time)
            if len(msg.buttons) > self.BTN_DPAD_DOWN and msg.buttons[self.BTN_DPAD_DOWN]:
                self.handle_speed_change(False, current_time)
            
            # Handle reset position (Select button)
            if len(msg.buttons) > self.BTN_SELECT and msg.buttons[self.BTN_SELECT]:
                self.handle_home_position(current_time)
            
            # Handle suction control (X button)
            if len(msg.buttons) > self.BTN_X and msg.buttons[self.BTN_X]:
                if current_time - self.last_button_press['suction'] > self.button_debounce_time:
                    try:
                        with self.device_lock:
                            self.suction_state = not self.suction_state
                            self.device.suck(self.suction_state)
                            self.get_logger().info(f'Suction turned {"on" if self.suction_state else "off"}')
                        self.last_button_press['suction'] = current_time
                    except Exception as e:
                        self.get_logger().error(f'Failed to control suction: {str(e)}')
            
            # Calculate movements based on control mode
            if self.control_mode == self.CONTROL_MODE_XYZ:
                # XYZ mode movement using right analog stick
                x_move = msg.axes[self.AXIS_RX] * self.linear_speed if abs(msg.axes[self.AXIS_RX]) > self.joy_deadzone else 0
                y_move = -msg.axes[self.AXIS_RY] * self.linear_speed if abs(msg.axes[self.AXIS_RY]) > self.joy_deadzone else 0
                
                # Z movement requires L1 (down) or R1 (up) buttons
                z_move = 0
                if len(msg.buttons) > self.BTN_R1:
                    if msg.buttons[self.BTN_R1]:  # Up
                        z_move = -self.z_speed
                        self.get_logger().info('Moving UP')
                    elif msg.buttons[self.BTN_L1]:  # Down
                        z_move = self.z_speed
                        self.get_logger().info('Moving DOWN')
                
                # Debug current position
                self.get_logger().info(f'Current position - X:{self.current_x:.1f} Y:{self.current_y:.1f} Z:{self.current_z:.1f} R:{self.current_r:.1f}')
                
                # Update target position
                self.target_x = self.current_x + x_move
                self.target_y = self.current_y + y_move
                self.target_z = self.current_z + z_move
                self.target_r = self.current_r  # No rotation movement
                
                # Send movement command if there's significant movement
                if abs(x_move) > 0.1 or abs(y_move) > 0.1 or abs(z_move) > 0.1:
                    self.get_logger().info(f'Sending move command - X:{self.target_x:.1f} Y:{self.target_y:.1f} Z:{self.target_z:.1f} R:{self.target_r:.1f}')
                    with self.device_lock:
                        self.device.move_to(self.target_x, self.target_y, self.target_z, self.target_r, wait=False)
                        self.get_logger().info('Move command sent successfully')
                    self.last_move_time = current_time
            
            # Handle suction control (X button for on, Triangle for options)
            if len(msg.buttons) > self.BTN_X and msg.buttons[self.BTN_X]:
                if current_time - self.last_button_press['suction'] > self.button_debounce_time:
                    with self.device_lock:
                        self.suction_state = not self.suction_state
                        self.device.suck(self.suction_state)
                        self.get_logger().info(f'Suction: {"On" if self.suction_state else "Off"}')
                    self.last_button_press['suction'] = current_time
            
            if len(msg.buttons) > self.BTN_TRIANGLE and msg.buttons[self.BTN_TRIANGLE]:
                if current_time - self.last_button_press['suction_options'] > self.button_debounce_time:
                    # Add suction options handling here if needed
                    self.get_logger().info('Suction options pressed')
                    self.last_button_press['suction_options'] = current_time
                    
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