import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import socket

class SteeringServoDriver():
    def __init__(self):
        # === CONSTANTS ===
        # Pulse width as measured from the RC car receiver in milliseconds
        self.TRIM = -50
        self.PW_LEFT = 1700 + self.TRIM #absolute left: 1980
        self.PW_RIGHT = 1300 + self.TRIM #absolute right: 1000
        self.PW_CENTER = 1500 + self.TRIM 
        
        # clamp response to achieve target
        self.current_pw = self.PW_CENTER
        self.current_steering = 0.0
        self.target_steering = 0.0
        self.MAX_STEERING_STEP = .1  # TODO find good value

    def step(self):
        # get steering value (-1,1) with a clamped step
        self.current_steering = np.clip(self.target_steering, self.current_steering -
                                        self.MAX_STEERING_STEP, self.current_steering + self.MAX_STEERING_STEP)
        self.current_steering = np.clip(self.current_steering,-1.0,1.0)

        # convert steering to duty cycle
        self.current_pw = self.PW_RIGHT + \
            (self.PW_LEFT - self.PW_RIGHT) * (.5*self.current_steering + .5)

        self.current_pw = np.clip(self.current_pw, min(
            self.PW_RIGHT, self.PW_LEFT), max(self.PW_RIGHT, self.PW_LEFT))

        return self.current_pw

    def setTargetSteering(self, steering):
        self.target_steering = np.clip(steering, -1.0, 1.0)


class MotorDriver():
    def __init__(self):
        # === CONSTANTS ===
        # Pulse width as measured from the RC car receiver in milliseconds
        self.TRIM = 0
        self.PW_BRAKE = 1600 + self.TRIM #absolute full brake = 1980
        self.PW_FULL_THROTTLE = 1400 + self.TRIM #absolute full throttle = 1000
        self.PW_NEUTRAL = 1500 + self.TRIM
        
        # clamp response to achieve target
        self.current_pw = self.PW_NEUTRAL
        self.current_throttle = 0.0
        self.target_throttle = 0.0
        self.MAX_THROTTLE_STEP = .1  # TODO find good value

        self.forward = True  # whether vehicle is in forward or reverse mode

    def Reverse(self):
        pass #TODO

    def Forward(self):
        pass #TODO

    def step(self):
        if(self.forward):
            # get thottle and brake values (-1,1) with a clamped step
            self.current_throttle = np.clip(self.target_throttle, self.current_throttle -
                                            self.MAX_THROTTLE_STEP, self.current_throttle + self.MAX_THROTTLE_STEP)
            self.current_throttle = np.clip(self.current_throttle, -1.0,1.0)
            # convert steering to duty cycle
            if(self.current_throttle >= 0):
                self.current_pw = self.PW_FULL_THROTTLE + \
                    (self.PW_NEUTRAL - self.PW_FULL_THROTTLE) * \
                    (1-self.current_throttle)
            else:
                self.current_pw = self.PW_NEUTRAL + \
                    (self.PW_BRAKE - self.PW_NEUTRAL) * \
                    -self.current_throttle

            self.current_pw = np.clip(
                self.current_pw, min(self.PW_BRAKE,self.PW_FULL_THROTTLE), max(self.PW_BRAKE,self.PW_FULL_THROTTLE))
            return self.current_pw

        else:
            pass

    def setTargetThrottle(self, throttle, braking):
        print("throttle: ", throttle)
        print("braking: ", braking)
        self.target_throttle = np.clip(throttle-braking, -1.0, 1.0)
        print(self.target_throttle)
        return self.target_throttle


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # update frequencies of this node
        self.freq = 10.0  # PWM is at 60Hz, so we should not overwrite previous signal too quickly

        # data that will be used by this class
        #self.stale_timer = 0  # will kill motors if no commands for specific amount of time
        #self.KILL_TIME = 0.3  # time after which motors will be killed if no new commands given
        
        # Setting up the UDP socket server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = ('0.0.0.0', 1209)
        self.sock.bind(self.server_address)
        
        # Setting up the loop to keep receiving data
        self.timer = self.create_timer(1/self.freq, self.receive_data_and_update_motors)

        # motor and servo objects
        self.motor = MotorDriver()
        self.servo = SteeringServoDriver()
        
        BAUD_RATE = 250000
        PORT = "/dev/ttyACM0"
        self.arduino = serial.Serial(port=PORT, baudrate=BAUD_RATE)

    def receive_data_and_update_motors(self):
        data, address = self.sock.recvfrom(12)  # 3 floats = 3 * 4 bytes = 12 bytes
        if data:
            steer, throttle, brake = struct.unpack('fff', data) 
            

            # ... [Rest of the update_motors method's content] ...
            #self.stale_timer += 1/self.freq
            #if(self.stale_timer >= self.KILL_TIME):
            #    throttle = 0.0
            #    brake = 0.0

            self.servo.setTargetSteering(steer)
            target = self.motor.setTargetThrottle(throttle, brake)
            servo_pw = int(self.servo.step())
            esc_pw = int(self.motor.step())
        
            msg_size = struct.pack("<B", 4)
            msg_data = struct.pack("<2H", *[servo_pw, esc_pw])
            msg = b"".join([msg_size, msg_data])
            t0 = time.time()
            self.arduino.write(msg)
            t1 = time.time()

def main(args=None):
    rclpy.init(args=args)
    driver = MotorDriverNode()
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

