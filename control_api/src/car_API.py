#!/usr/bin/env python3
import serial
import struct
import rospy
from std_msgs.msg import Float64

class ControlCar():
    def __init__(self, baudrate = 5000000) -> None:
        rospy.init_node("car_API")
        rospy.Subscriber("/SteeringAngle",Float64,self.recieveSteeringCommands)
        rospy.Subscriber("/cmd_vel",Float64,self.recieveVelocityCommands)
        self.angle_pub = rospy.Publisher('/angle', Float64, queue_size = 1)
        self.delta_angle_pub = rospy.Publisher('/delta_angle', Float64, queue_size = 1)
        self.steering_speed_pub = rospy.Publisher('/steering_speed', Float64, queue_size = 1)
        
        self.ser = serial.Serial('/dev/ttyUSB0', baudrate)
        #input to the car
        self.speed_car = 0.0 
        self.steer_angle_input = 0.0 #degree

    def collectSteeringAngle(self) -> float:
        num_elements = 8
        response = self.ser.read(num_elements*4) # Read 4 bytes per element (assuming 32-bit float)
        if response:
            vector_data = list(struct.unpack('<' + 'f'*num_elements, response)) # Convert bytes to vector
        return vector_data[1]
    
    def sendCommandTocar(self,steering = 0.0 , velocity = 0.0):
        vector_data = [velocity, steering]  # [velocity, steering]
        num_elements = len(vector_data)
        bytes_data = struct.pack('<' + 'f'*num_elements, *vector_data) # Convert vector to bytes
        self.ser.write(bytes_data) # Send bytes over serial
    def recieveSteeringCommands(self,msg):
        """
        Send steering c
        """
        self.steer_angle_input = msg.data
        if self.steer_angle_input > 28.8 :
            self.steer_angle_input = 28.0
        elif self.steer_angle_input < -28.0:
            self.steer_angle_input = -28.0
        
        if self.steer_angle_input <= 13 and self.steer_angle_input >= 10:
            self.steer_angle_input = 16
        elif self.steer_angle_input >= -13 and self.steer_angle_input <= -10:
            self.steer_angle_input = -16

        self.sendCommandTocar (steering = self.steer_angle_input , velocity = self.speed_car)

    def recieveVelocityCommands(self,msg:Float64):
        """
        Send velocity commands to the car
        """
        self.speed_car = msg.data
        if self.speed_car < 0.0:
            self.speed_car = 0.0
        self.sendCommandTocar (steering = self.steer_angle_input , velocity = self.speed_car)

if __name__ == "__main__":

    matgr = ControlCar()

    while not rospy.is_shutdown():
        print(f"steering angle = {matgr.collectSteeringAngle()} degree, velocity = {matgr.speed_car} km/h")

    
    
    
    
    
