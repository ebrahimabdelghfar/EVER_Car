import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes and provides convenience functions
from ackermann_msgs.msg import AckermannDrive # Standard ROS 2 message type and import it into Python
import serial
import struct

class MatgrCar(Node):
    def __init__(self):
        # super().__init__('MatgrCar')
        #connect to the car
        self.ser = serial.Serial('/dev/ttyUSB0', 5000000)
        # self.publisher_ = self.create_subscription(AckermannDrive, '/matgrCarController',self.moveCar, 10)
        # self.create_timer(0.1, self.readSteeringAngle)

    def collectSteeringAngle(self) -> float:
        """
        Parameters:
        -----------
            None
        returns:
        --------
            steering angle in degree
        """
        num_elements = 8
        response = self.ser.read(num_elements*4) # Read 4 bytes per element (assuming 32-bit float)
        if response:
            vector_data = list(struct.unpack('<' + 'f'*num_elements, response)) # Convert bytes to vector
        return vector_data[1]
    
    def sendCommandTocar(self,steering:float = 0.0 , velocity:float = 0.0):
        """
        Parameters:
        -----------
            steering : float
                The steering angle in degree
            velocity : float
                The velocity of the car in km/h
        returns:
        --------
            None
        """
        #define the range of the steering angle
        if steering > 28.8 :
            steering = 28.0
        elif steering < -28.0:
            steering = -28.0
        
        # those conditions was declared as the angle between 10 and 13 degree is not stable
        if steering <= 13 and steering >= 10:
            steering = 18 # the 18 was chosen as to skip the range between 10 and 13 degree quikly due to the effect of P 
        elif steering >= -13 and steering <= -10:
            steering = -18

        vector_data = [velocity, steering]  # [velocity, steering]
        num_elements = len(vector_data)
        bytes_data = struct.pack('<' + 'f'*num_elements, *vector_data) # Convert vector to bytes
        self.ser.write(bytes_data) # Send bytes over serial

    def readSteeringAngle(self):
        steering_angle = self.collectSteeringAngle()
        self.get_logger().info('Steering angle: "%s"' % steering_angle)

    def moveCar(self, msg: AckermannDrive):
        self.get_logger().info('Publishing: "%s"' % msg)
        #m/sec to km/h
        msg.speed = msg.speed * 3.6
        self.sendCommandTocar(steering=msg.steering_angle, velocity=msg.speed)
     
def main(args=None):
    rclpy.init(args=args)
    matgr = MatgrCar()
    rclpy.spin(matgr)
    rclpy.shutdown()
