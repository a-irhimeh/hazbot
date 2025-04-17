import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Serial connection
        self.serialPort = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.5)

        # Motor specs
        self.max_speed = 20  # Max value for motor controller

        self.get_logger().info("Motor Controller node started.")

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x      # Forward/backward
        angular = msg.angular.z    # Left/right turn

        # Differential drive calculations
        left_speed = linear - angular
        right_speed = linear + angular

        # Normalize to -1.0 to 1.0 range (assuming max linear speed = 1.0)
        max_val = max(abs(left_speed), abs(right_speed), 1.0)
        left_norm = left_speed / max_val
        right_norm = right_speed / max_val

        # Convert to motor values (0–63 and direction)
        def calc_motor(val):
            direction = 1 if val >= 0 else 0  # 1 = forward, 0 = backward
            speed = int(abs(val) * self.max_speed)
            return direction, speed

        directionL, speedL = calc_motor(left_norm)
        directionR, speedR = calc_motor(right_norm)

        # Send to motor driver
        self.set_motor(directionL, speedL, directionR, speedR)

    def set_motor(self, directionL, speedL, directionR, speedR):
        speedL = min(max(speedL, 0), 63)
        speedR = min(max(speedR, 0), 63)

        directionBitL = 64 if directionL == 0 else 0
        directionBitR = 64 if directionR == 0 else 0

        motorLeft = speedL | directionBitL
        motorRight = (speedR | directionBitR) | 128  # Set bit 7 for right motor

        packet = bytearray([motorLeft, motorRight])
        self.serialPort.write(packet)

    def destroy_node(self):
        # Stop motors and close serial on shutdown
        self.set_motor(0, 0, 0, 0)
        self.serialPort.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
