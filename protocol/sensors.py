import enum
class Port(enum.Enum):
    A = 0
    B = 1
    C = 2
    D = 3

class MotorTicks:
    def __init__(self, port, ticks):
        self.port = port #enum:Port
        self.ticks = ticks #int

    def to_dict(self):
        ret = dict()
        ret['port'] = self.port.name
        ret['ticks'] = self.ticks
        return ret

    def get_topic(self):
        return 'brickcontrol/sensor/motor_ticks'

    def get_topic_static():
        return 'brickcontrol/sensor/motor_ticks'

    def from_dict(input_dict):
        port = Port[input_dict['port']]
        ticks = input_dict['ticks']
        return MotorTicks(
            port,
            ticks,
        )

class MotorOdometry:
    def __init__(self, motor_ticks):
        self.motor_ticks = motor_ticks #list:struct:MotorTicks

    def to_dict(self):
        ret = dict()
        ret['motor_ticks'] = []
        for data_point in self.motor_ticks:
            ret['motor_ticks'].append(data_point.to_dict())
        return ret

    def get_topic(self):
        return 'brickcontrol/sensor/motor_odometry'

    def get_topic_static():
        return 'brickcontrol/sensor/motor_odometry'

    def from_dict(input_dict):
        motor_ticks = []
        for data_point in input_dict['motor_ticks']:
            motor_ticks.append(MotorTicks.from_dict(data_point))
        return MotorOdometry(
            motor_ticks,
        )

class ProximityMeasurement:
    def __init__(self, distance):
        self.distance = distance #int

    def to_dict(self):
        ret = dict()
        ret['distance'] = self.distance
        return ret

    def get_topic(self):
        return 'brickcontrol/sensor/proximity_measurement'

    def get_topic_static():
        return 'brickcontrol/sensor/proximity_measurement'

    def from_dict(input_dict):
        distance = input_dict['distance']
        return ProximityMeasurement(
            distance,
        )

class SensorMessage:
    def __init__(self, timestamp, movement_vector, proximity_measurement):
        self.timestamp = timestamp #int
        self.movement_vector = movement_vector #list:double
        self.proximity_measurement = proximity_measurement #struct:ProximityMeasurement

    def to_dict(self):
        ret = dict()
        ret['timestamp'] = self.timestamp
        ret['movement_vector'] = []
        for data_point in self.movement_vector:
            ret['movement_vector'].append(data_point)
        ret['proximity_measurement'] = self.proximity_measurement.to_dict()
        return ret

    def get_topic(self):
        return 'brickcontrol/sensor/inc_sensor_data'

    def get_topic_static():
        return 'brickcontrol/sensor/inc_sensor_data'

    def from_dict(input_dict):
        timestamp = input_dict['timestamp']
        movement_vector = []
        for data_point in input_dict['movement_vector']:
            movement_vector.append(data_point)
        proximity_measurement = ProximityMeasurement.from_dict(input_dict['proximity_measurement'])
        return SensorMessage(
            timestamp,
            movement_vector,
            proximity_measurement,
        )

