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

# Encoder Ticks / 90 degrees
jointTicks = [47000, 47000, 47000, -1, -1, -1]

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
                posArr = []
                speedArr = []
                for x in range(count):
                    ix1 = 2 + x * 6
                    ix2 = ix1 + 4
                    pos = int.from_bytes(
                        data[ix1:ix2], byteorder='little', signed=True)
                    speed = int.from_bytes(
                        data[ix2:ix2 + 2], byteorder='little', signed=True)
                    posArr.append(pos)
                    speedArr.append(speed)

                return StatusMessage(posArr, speedArr)
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

    def speed(self, ix, speed):
        ix_bytes = ix.to_bytes(1, byteorder='little', signed=True)
        speed_bytes = speed.to_bytes(2, byteorder='little', signed=True)
        self.send(speedCmd + ix_bytes + speed_bytes)
        return self.receive()

    def position(self, ix, pos):
        ix_bytes = ix.to_bytes(1, byteorder='little', signed=True)
        pos_bytes = pos.to_bytes(4, byteorder='little', signed=True)
        self.send(positionCmd + ix_bytes + pos_bytes)
        return self.receive()
    
    def angle(self, ix, degrees):
        if degrees < 0 or degrees > 180:
            raise "Invalid angle: " + degrees
        if jointTicks[ix] <= 0:
            raise "Joint not configured!"
            
        ticks = int(degrees * jointTicks[ix] / 90.0)

        self.position(ix, ticks)
    
