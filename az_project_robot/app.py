from flask import Flask, render_template, request, Response, jsonify
import cv2
import numpy as np
from time import sleep, time
from src.robot.modes import Modes
from src.hardware.motors import Motors  # Importing Motors to control the motors

app = Flask(__name__)

# Camera
camera = cv2.VideoCapture(0)

# Tạo các khung hình phát trực tiếp
def gen_frames(source='camera'):
    while True:
        success, frame = camera.read()
        if not success:
            continue

        buffer = None
        if source == 'camera':
            ret, buffer = cv2.imencode('.jpg', frame)
        
        if buffer is None or not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/manual_control', methods=['POST'])
def manual_control_route():
    data = request.json
    command = data.get('command')
    
    # Create a Modes object and handle the command
    mode = Modes()
    # Assuming you have a method to handle the command and update the robot's movement
    mode.manual_control(command)

    return {'status': 'command_received', 'command': command}

@app.route('/status')
def get_status():
    # Example status data, this would typically be dynamic, based on the robot's current state
    # You can use actual robot status variables from your robot's sensors
    robot_status = {
        'battery': 85,          # Example battery percentage
        'waterLevel': 50,       # Example water level
        'robotStatus': 'Idle',  # Example robot status
        'wheelSpeed': 0.5       # Example wheel speed
    }
    return jsonify(robot_status)

@app.route('/video_feed/<source>')
def video_feed(source):
    return Response(gen_frames(source), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
