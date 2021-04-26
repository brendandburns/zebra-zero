from logging import error
import serial
from cobs import cobs
import math

zeroByte = b'\x00'

debugCmd = b'\x00'
statusCmd = b'\x01'
stopCmd = b'\x02'
rawCmd = b'\x03'
speedCmd = b'\x04'
positionCmd = b'\x05'
gripperOpenCmd = b'\x06'
gripperCloseCmd = b'\x07'
gripperStatusCmd = b'\x08'


class DebugMessage:
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message


class StatusMessage:
    def __init__(self, posArr, speedArr):
        self.pos = posArr
        self.speed = speedArr

    def __str__(self):
        return 'Position ({}) - Velocity ({})'.format(self.pos, self.speed)


class GripperStatusMessage:
    def __init__(self, left_gripper, right_gripper, top_switch, bottom_switch, gripper_switch, mode):
        self.left_gripper = left_gripper
        self.right_gripper = right_gripper
        self.top_switch = top_switch
        self.bottom_switch = bottom_switch
        self.gripper_switch = gripper_switch
        self.mode = mode

    def __str__(self):
        return 'Bumpers ({}, {}) - Gripper ({}, {}, {}) - Mode: {}'.format(
            self.left_gripper, self.right_gripper, self.top_switch, self.bottom_switch, self.gripper_switch, self.mode)


class ClientError(Exception):
    def __init__(self, msg):
        self.msg = msg
    
    def __str__(self):
        return self.msg


class RobotClient:
    def __init__(self, path, serialObj=None):
        if serialObj is not None:
            self.ser = serialObj
        else:
            self.ser = serial.Serial(
                path, baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=3)
        self.WHEEL_RADIUS = 6.25 / 100
        self.WHEEL_CIRCUMFERENCE = self.WHEEL_RADIUS * 2 * math.pi
        # The circle described by the robot rotating in place
        self.ROBOT_CIRCUMFERENCE = 16 / 100 * 2 * math.pi 
        self.CLICKS_PER_ROTATION = 8192
    
    def init(self):
        self.ser.read_until(zeroByte)

    def receive(self):
        data = self.ser.read_until(zeroByte)
        n = len(data)
        if n > 0:
            # clip last byte & decode
            data = cobs.decode(data[0:(n-1)])
            if data[0] == 0:
                return DebugMessage(data[1:].decode('utf-8'))
            if data[0] == 1:
                count = data[1]
                pos = int.from_bytes(
                    data[2:6], byteorder='little', signed=True)
                speed = int.from_bytes(
                    data[6:8], byteorder='little', signed=True)

                return StatusMessage([pos], [speed])
            if data[0] == 8:
                leftGripper = data[1] == 1
                rightGripper = data[2] == 1
                topSwitch = data[3] == 1
                bottomSwitch = data[4] == 1
                gripperSwitch = data[5] == 1
                gripperMode = data[6]
                
                return GripperStatusMessage(leftGripper, rightGripper, topSwitch, bottomSwitch, gripperSwitch, gripperMode)

            raise "Unknown header: {}".format(data[0])

    def send(self, data):
        self.ser.write(cobs.encode(data))
        self.ser.write(zeroByte)

    def status(self):
        self.send(statusCmd)
        return self.receive()
    
    def debug(self):
        self.send(debugCmd)
        return self.receive()

    def stop(self):
        self.send(stopCmd)
        return self.receive()

    def raw(self, ix, speed):
        ix_bytes = ix.to_bytes(1, byteorder='little', signed=True)
        speed_bytes = speed.to_bytes(1, byteorder='little', signed=True)
        self.send(rawCmd + ix_bytes + speed_bytes)
        return self.receive()

    def speed(self, rightSpeed, leftSpeed):
        left_bytes = leftSpeed.to_bytes(2, byteorder='little', signed=True)
        right_bytes = rightSpeed.to_bytes(2, byteorder='little', signed=True)
        self.send(speedCmd + left_bytes + right_bytes)
        return self.receive()

    def position(self, rightPos, leftPos):
        left_bytes = leftPos.to_bytes(4, byteorder='little', signed=True)
        right_bytes = rightPos.to_bytes(4, byteorder='little', signed=True)
        self.send(positionCmd + left_bytes + right_bytes)
        return self.receive()
    
    def gripper_open(self):
        self.send(gripperOpenCmd)
        return self.receive()
    
    def gripper_close(self):
        self.send(gripperCloseCmd)
        return self.receive()
    
    def move_meters(self, meters):
        current = self.stop()
        clicks = self.CLICKS_PER_ROTATION * meters / self.WHEEL_CIRCUMFERENCE
        return self.position(int(current.right_pos + clicks), int(current.left_pos + clicks))

    def forward(self, meters):
        return self.move_meters(-meters)
    
    def backward(self, meters):
        return self.move_meters(meters)

    def rotate(self, degrees):
        current = self.stop()
        distance = self.ROBOT_CIRCUMFERENCE * degrees / 360.0
        print()
        clicks = self.CLICKS_PER_ROTATION * distance / self.WHEEL_CIRCUMFERENCE
        return self.position(int(current.right_pos + clicks), int(current.left_pos - clicks))
    
    def say(self, message):
        self.speech.say(message)
    
    def camera(self):
        return self.camera.snapshot()