from flask import Flask, render_template, request, Response, jsonify
import cv2
from src.robot.modes import Modes
from src.utils.control_utils import set_motors_direction, direction
from src.hardware.water_sensor import WaterLevelSensor
from src.hardware.battery import BatteryMonitor
from src.hardware.ultrasonic import UltrasonicSensors
from src.vision.color_detection import color_detection_loop
from src.vision.object_detection import object_detection_loop
from src.vision.video_stream import VideoStream
from queue import Queue
from threading import Thread, Event
import time
from hand_control import manual_control, update_state, increase_speed, decrease_speed, move_robot, toggle_relay, control_servo, stop_robot

app = Flask(__name__)
modes = Modes()  # Khởi tạo instance


@app.route('/')
def index():
    return render_template('index.html')  




if __name__ == '__main__':

    app.run(host='0.0.0.0', port=5000)
