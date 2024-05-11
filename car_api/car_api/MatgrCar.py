import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes and provides convenience functions
from ackermann_msgs.msg import AckermannDrive # Standard ROS 2 message type and import it into Python
import serial
import struct

class MatgrCar(Node):
    def __init__(self,NODE_NAME = "MatgrCar",PORT:str = '/dev/ttyUSB1', BAUDRATE:int = 5000000):
        super().__init__(NODE_NAME)
        #connect to the car
        self.ser = serial.Serial(PORT, BAUDRATE)

        self.publisher_ = self.create_subscription(msg_type = AckermannDrive,
                                                   topic = '/matgrCarController',
                                                   callback = self.moveCar,
                                                   qos_profile = 10)
    
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
            steering = 15 # the 15 was chosen as to skip the range between 10 and 13 degree quikly due to the effect of P 
        elif steering >= -13 and steering <= -10:
            steering = -15

        vector_data = [velocity, steering]  # [velocity, steering]
        num_elements = len(vector_data)
        bytes_data = struct.pack('<' + 'f'*num_elements, *vector_data) # Convert vector to bytes
        self.ser.write(bytes_data) # Send bytes over serial

    def moveCar(self, msg: AckermannDrive):
        #m/sec to km/h
        msg.speed = msg.speed * 3.6
        #rad to degree
        msg.steering_angle = msg.steering_angle * 57.2958
        self.sendCommandTocar(steering=msg.steering_angle, velocity=msg.speed)
     
def main(args=None):
    rclpy.init(args=args)
    matgr = MatgrCar()
    rclpy.spin(matgr)
    rclpy.shutdown()