import numpy as np
import paho.mqtt.client as mqtt
from time import sleep
import json
from protocol.sensors import *
import matplotlib.pyplot as plt
import queue
from odometry.odometry import DeadReckoner, ticks_to_movement_vector
from robot_math.transformations import *

WHEEL_BASE = 163
WHEEL_DIAMETER = 56

TICKS_PER_ROTATION = 360

WHEEL_PERIMETER = WHEEL_DIAMETER*np.pi


x_pos = []
y_pos = []

event_queue = queue.Queue()
def on_message(client, userdata, message):
    event_queue.put(message.payload)


client =mqtt.Client()
client.on_message = on_message
client.connect("localhost")

client.subscribe(SensorMessage.get_topic_static())
client.loop_start()


x_pos = []
y_pos = []
cnt = 0

odo_builder = DeadReckoner(WHEEL_BASE, WHEEL_PERIMETER, TICKS_PER_ROTATION, Port.D, Port.A)

sensor_pose = np.asarray([-110, -110, -np.pi/2])
sensor_x = []
sensor_y = []
while True:
    e = event_queue.get(block=True)
    sensor_msg  = SensorMessage.from_dict(json.loads(e))
    odo_builder.update_position(np.asarray(sensor_msg.movement_vector))

    for dist in sensor_msg.proximity_measurement.distance:
        pt = np.asarray([dist,0,0])

        loc_pt = loc2glob_pose(pt, sensor_pose)
        
        glob_pt = loc2glob_pose(loc_pt, odo_builder.position)
        sensor_x.append(glob_pt[0])
        sensor_y.append(glob_pt[1])

    #print(odo_builder.position[2])
    cnt = cnt+1
    
    if cnt == 10:
        position = odo_builder.position
        cnt = 0
        x_pos.append(position[0])
        y_pos.append(position[1])
        #print(len(x_pos))
        plt.clf()
        plt.plot(x_pos, y_pos, 'r.')
        plt.plot(sensor_x, sensor_y, 'b.')
        plt.axis("equal")
        plt.ion()
        plt.show()
        plt.pause(0.001)

        if len(x_pos) > 1000:
            x_pos = x_pos[100:]
            y_pos = y_pos[100:]

        if len(sensor_x) > 10000:
            sensor_x = sensor_x[100:]
            sensor_y = sensor_y[100:] 

    print(odo_builder.position)