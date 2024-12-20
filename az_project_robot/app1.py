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
from manual_control import manual_control, update_state, increase_speed, decrease_speed, move_robot, toggle_relay, control_servo, stop_robot

app = Flask(__name__)
modes = Modes()  # Khởi tạo instance
state = "idle"

speed_level = 5  # Default speed level (range: 0 to 10)
speed_factor = 0.04309596457  # Multiplier for converting speed level to actual speed
vx = vy = speed_level * speed_factor
@app.route('/')
def index():
    return render_template('index.html')  

@app.route('/control', methods=['POST'])
def manual_control_api():
    global state, speed_level, vx, vy
    data = request.json
    command = data.get('command', None)

    if not command:
        return jsonify({"error": "No command provided"}), 400

    if command in ['+', '=', '-', '_', 'w', 'a', 's', 'd', 'q', 'e', 'z', 'x', 'r', '7', '8', '9', '0', '1','2','3']:
        if command == '+':
            speed_level = min(speed_level + 1, 10)
            vx = vy = speed_level * speed_factor
            state = f"speed increased to {speed_level * 10}%"
        elif command  == '-':
            speed_level = max(speed_level - 1, 0)
            vx = vy = speed_level * speed_factor
            state = f"speed decreased to {speed_level * 10}%"
        elif command in ['w', 'a', 's', 'd', 'q', 'e', 'z', 'x','1','2','3']:
            move_robot(command)
        elif command == 'r':
            toggle_relay()
        elif command in ['7', '8', '9', '0']:
            control_servo(command)
        return jsonify({"status": "Command executed successfully", "current_state": state})

    return jsonify({"error": "Invalid command"}), 400


# @app.route('/video_feed/color')
# def video_feed_color_detection():
#     return Response(gen_frames("color"), mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/video_feed/object')
# def video_feed_object_detection():
#     return Response(gen_frames("object"), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
