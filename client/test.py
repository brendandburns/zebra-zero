from client import StatusMessage, RobotClient
from cobs import cobs
import unittest
from math import pi
from arm import Arm

class MockSerial:
    def __init__(self):
        self.bytes = []
    
    def write(self, bytes):
        self.bytes += bytes
    
    def read_until(self, stop):
        return self.read

    def set_status(self, positions, speeds):
        msg = b'\x01'
        msg += len(positions).to_bytes(1, byteorder='little', signed=True)
        for ix in range(len(positions)):
            msg += positions[ix].to_bytes(4, byteorder='little', signed=True)
            msg += speeds[ix].to_bytes(2, byteorder='little', signed=True)
        encoded = cobs.encode(msg) + b'\x00'
        self.read = encoded


class TestClient(unittest.TestCase):

    def test_conversion_round_trip(self):
        angles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        encoders = Arm.radians_to_encoder(angles)
        anglesOut = Arm.encoder_to_radians(encoders)
        for ix in range(len(angles)):
            self.assertAlmostEqual(angles[ix], anglesOut[ix], 4, 'Angles should be equal ({})'.format(ix))

    
    def test_to_degrees(self):
        radians = [pi / 2, 3 * pi / 2, 2 * pi, pi / 4, pi, 4 * pi / 3]
        angles = [90, 270, 360, 45, 180, 240]
        
        encoders = Arm.radians_to_encoder(radians)
        anglesOut = Arm.encoder_to_degrees(encoders)

        for ix in range(len(angles)):
            self.assertAlmostEqual(angles[ix], anglesOut[ix], 2, 'Angles should be equal ({})'.format(ix))
    

    def test_encoder_limits_bad(self):
        self.assertRaises(ValueError, lambda: Arm.encoders_within_limits([1000000, 0, 0, 0, 0, 0]))


    def test_encoder_limits_ok(self):
        # validate this doesn't raise an exception
        Arm.encoders_within_limits([0, 0, 0, 0, 0, 0])


    def test_angle_limits_bad(self):
        self.assertRaises(ValueError, lambda: Arm.degrees_within_limits([1000000, 0, 0, 0, 0, 0]))


    def test_angle_limits_ok(self):
        # validate this doesn't raise an exception
        Arm.degrees_within_limits([0, 0, 0, 0, 0, 0])
    

    def test_status(self):
        serial = MockSerial()
        client = RobotClient('/dev/ttyFoo', serial)
        serial.set_status([0, 100, 200], [3, 4, 5])
        res = client.status()
        self.assertEqual(len(res.pos), 6)
        self.assertEqual(len(res.speed), 6)
        self.assertEqual(res.pos[0], 0)
        self.assertEqual(res.pos[1], 100)
        self.assertEqual(res.pos[2], 200)
        self.assertEqual(res.speed[0], 3)
        self.assertEqual(res.speed[1], 4)
        self.assertEqual(res.speed[2], 5)


if __name__ == '__main__':
    unittest.main()