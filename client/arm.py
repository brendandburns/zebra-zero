from math import degrees, pi, radians

class Arm:
    ANGLE_AT_HOME = [
        0,
        0,
        0,
        0,
        pi,
        0,
    ]

    LIMITS_DEGREES = [
        [-180, 180],
        [-30, 200],
        [-30, 100],
        # TODO: Fix these
        [-180, 180],
        [-180, 180],
        [-180, 180]
    ]
    LIMITS_RADIANS = list(map(lambda x: [radians(x[0]), radians(x[1])], LIMITS_DEGREES))

    LIMITS_ENCODERS = [
        [-98000, 98000],
        [-16000, 100000],
        [-15000, 59000],
        [0, 0],
        [0, 0],
        [0, 0]
    ]

    @staticmethod
    def encoder_within_limits(ix, encoder):
        if encoder < Arm.LIMITS_ENCODERS[ix][0] or encoder > Arm.LIMITS_ENCODERS[ix][1]:
                raise ValueError('Encoder out of range: {} ({}) @ {}'.format(encoder, Arm.LIMITS_ENCODERS[ix], ix))

    @staticmethod
    def encoders_within_limits(encoders):
        for ix in range(len(encoders)):
            Arm.encoder_within_limits(ix, encoders[ix])

    @staticmethod
    def degree_within_limits(ix, degree):
        if degree < Arm.LIMITS_DEGREES[ix][0] or degree > Arm.LIMITS_DEGREES[ix][1]:
            raise ValueError('Degrees out of range: {} ({}) @ {}'.format(degree, Arm.LIMITS_DEGREES[ix], ix))
    
    @staticmethod
    def degrees_within_limits(degrees):
        for ix in range(len(degrees)):
            Arm.degree_within_limits(ix, degrees[ix])

    @staticmethod
    def radians_within_limits(radians):
        for ix in range(len(radians)):
            if radians[ix] < Arm.LIMITS_RADIANS[ix][0] or radians[ix] > Arm.LIMITS_RADIANS[ix][1]:
                raise ValueError('Radians out of range: {} ({}) @ {}'.format(radians[ix], Arm.LIMITS_RADIANS[ix], ix))

    @staticmethod
    def radians_to_encoder(angles):
        encoder = [0, 0, 0, 0, 0, 0]

        encoder[0] = (angles[0] - Arm.ANGLE_AT_HOME[0]) * 30557.75
        encoder[1] = (angles[1] - Arm.ANGLE_AT_HOME[1]) * 30557.75
        encoder[2] = (angles[2] - Arm.ANGLE_AT_HOME[2]) * 30557.75

        temp3 = 16551.79 * (angles[2] - Arm.ANGLE_AT_HOME[2])
        temp4 = 11034.53 * (angles[3] - Arm.ANGLE_AT_HOME[3])

        encoder[3] = temp3 + temp4
        encoder[4] = -temp3 + temp4 + 22069.06 * (angles[4] - Arm.ANGLE_AT_HOME[4]) - 11034.53 * (angles[5] - Arm.ANGLE_AT_HOME[5])
        encoder[5] = -temp3 + 11034.53 * (angles[4] - Arm.ANGLE_AT_HOME[4]) + 5517.265 * (Arm.ANGLE_AT_HOME[3] - angles[3] + angles[5] - Arm.ANGLE_AT_HOME[5])

        encoder[3] = encoder[3] * 24.0 / 17.33
        encoder[4] = encoder[4] * 24.0 / 17.33
        encoder[5] = encoder[5] * 24.0 / 17.33

        return list(map(int, encoder))
    
    @staticmethod
    def encoder_to_radians(encoders):
        angles = [0, 0, 0, 0, 0, 0]

        angles[0] = encoders[0] / 30557.75 + Arm.ANGLE_AT_HOME[0]
        angles[1] = encoders[1] / 30557.75 + Arm.ANGLE_AT_HOME[1]
        angles[2] = encoders[2] / 30557.75 + Arm.ANGLE_AT_HOME[2]

        ec3 = encoders[3] * 17.33 / 24.0
        ec4 = encoders[4] * 17.33 / 24.0
        ec5 = encoders[5] * 17.33 / 24.0

        angles[3] = ec3 / 11034.53 - encoders[2] / 20371.83 + Arm.ANGLE_AT_HOME[3]
        angles[4] = ec5 / 22069.06 + ec4 / 44138.12 + encoders[2] / 27162.44 + Arm.ANGLE_AT_HOME[4]
        angles[5] = ec5 / 11034.53 - ec4 / 22069.06 + ec3 / 11034.53 - encoders[2] / 40743.68 + Arm.ANGLE_AT_HOME[5]

        return angles
    
    @staticmethod
    def encoder_to_degrees(encoders):
        rads = Arm.encoder_to_radians(encoders)
        return list(map(degrees, rads))
    
    @staticmethod
    def degrees_to_encoder(angles):
        rads = list(map(radians, angles))
        return Arm.radians_to_encoder(rads)
