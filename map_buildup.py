import numpy as np
import paho.mqtt.client as mqtt
from time import sleep
import json
from protocol.sensors import *
import matplotlib.pyplot as plt
import queue
from odometry.odometry import DeadReckoner, ticks_to_movement_vector

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
client.connect("10.0.0.1")

client.subscribe("brickcontrol/sensor/motor_odometry")
client.loop_start()
left_encoder = None
right_encoder = None

x_pos = []
y_pos = []
cnt = 0

odo_builder = DeadReckoner(WHEEL_BASE, WHEEL_PERIMETER, TICKS_PER_ROTATION, Port.D, Port.A)

while True:
    e = event_queue.get(block=True)
    odo  = MotorOdometry.from_dict(json.loads(e))
    odo_builder.update_position(odo)
    #print(odo_builder.position[2])
    cnt = cnt+1
    
    if cnt == 10:
        position = odo_builder.position
        cnt = 0
        x_pos.append(position[0])
        y_pos.append(position[1])
        #print(len(x_pos))
        plt.clf()
        plt.plot(x_pos, y_pos, 'b.')
        plt.axis("equal")
        plt.ion()
        plt.show()
        plt.pause(0.001)

        if len(x_pos) > 1000:
            x_pos = x_pos[100:]
            y_pos = y_pos[100:]

    print(odo_builder.position)