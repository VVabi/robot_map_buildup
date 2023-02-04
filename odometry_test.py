import numpy as np
import paho.mqtt.client as mqtt
from time import sleep
import json
from protocol.sensors import *
import matplotlib.pyplot as plt
import queue

WHEEL_BASE = 163
WHEEL_DIAMETER = 56

TICKS_PER_ROTATION = 360

WHEEL_PERIMETER = WHEEL_DIAMETER*np.pi


left_encoder = None
right_encoder = None

position = [0, 0, 0]

x_pos = []
y_pos = []

event_queue = queue.Queue()
def on_message(client, userdata, message):
    event_queue.put(message.payload)


def update_position(position, left_encoder, right_encoder, motor_odometry):
    if left_encoder is None:
        left_encoder  = motor_odometry.motor_ticks[1].ticks
        right_encoder = motor_odometry.motor_ticks[0].ticks
        return position, left_encoder, right_encoder

    new_left_encoder  = motor_odometry.motor_ticks[1].ticks
    new_right_encoder = motor_odometry.motor_ticks[0].ticks

    left_dist = (new_left_encoder-left_encoder)/TICKS_PER_ROTATION*WHEEL_PERIMETER
    right_dist = (new_right_encoder-right_encoder)/TICKS_PER_ROTATION*WHEEL_PERIMETER

    right_encoder = new_right_encoder
    left_encoder = new_left_encoder

    trans_dist  = (left_dist+right_dist)/2
    angle       = (left_dist-right_dist)/WHEEL_BASE

    theta = position[2]+angle

    while theta > np.pi:
        theta = theta - 2*np.pi
    while theta < -np.pi:
        theta = theta + 2*np.pi

    x = position[0]+trans_dist*np.cos(position[2]+0.5*angle)
    y = position[1]+trans_dist*np.sin(position[2]+0.5*angle)
    position = [x,y, theta]

    x_pos.append(x)
    y_pos.append(y)

    print(position)

    return position, left_encoder, right_encoder

client =mqtt.Client()
client.on_message = on_message
client.connect("10.0.0.1")

client.subscribe("brickcontrol/sensor/motor_odometry")
client.loop_start()
left_encoder = None
right_encoder = None
position = [0,0,0]

x_pos = []
y_pos = []
cnt = 0
while True:
    e = event_queue.get(block=True)

    position, left_encoder, right_encoder = update_position(position, left_encoder, right_encoder, MotorOdometry.from_dict(json.loads(e)))
    cnt = cnt+1

    if cnt == 10:
        cnt = 0
        x_pos.append(position[0])
        y_pos.append(position[1])
        print(len(x_pos))
        plt.clf()
        plt.plot(x_pos, y_pos, 'b.')
        plt.axis("equal")
        plt.ion()
        plt.show()
        plt.pause(0.001)

        if len(x_pos) > 1000:
            x_pos = x_pos[100:]
            y_pos = y_pos[100:]