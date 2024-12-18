from flask import Flask, render_template, request, Response, jsonify
import cv2
from src.robot.modes import Modes
from src.utils.control_utils import set_motors_direction
from src.hardware.water_sensor import WaterLevelSensor
from src.hardware.battery import BatteryMonitor
from src.hardware.ultrasonic import UltrasonicSensors

app = Flask(__name__)

camera = cv2.VideoCapture(0)  # Khởi tạo camera
robot_modes = Modes()  # Khởi tạo đối tượng quản lý robot
water_level = WaterLevelSensor()
battery_level = BatteryMonitor()
ultrasonic_sensor = UltrasonicSensors()

def gen_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        _, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')  

@app.route('/manual_control', methods=['POST'])
def manual_control():
    data = request.json
    command = data.get('command', None)

    if not command:
        return jsonify({"error": "No command provided"}), 400

    response = robot_modes.execute_command(command)
    set_motors_direction(command, robot_modes.vx, robot_modes.vy, robot_modes.theta)
    
    return jsonify(response)

@app.route('/status')
def get_status():
    # Gọi hàm read_battery_status để lấy các thông tin về pin
    voltage, current, power, battery_percentage, remaining_time_hours = battery_level.read_battery_status()
    front_distance = ultrasonic_sensor.get_distance("front")
    left_distance = ultrasonic_sensor.get_distance("left")
    right_distance = ultrasonic_sensor.get_distance("right")
    water_present = water_level.is_water_present()

    # Tạo dictionary status với các thông tin khác
    status = {
        "voltage": voltage,
        "current": current,
        "power": power,
        "percentage": battery_percentage,
        "remaining_time": remaining_time_hours,

        "water": "Có nước" if water_present else "Khát quá"
        "front_sensor": front_distance,
        "left_sensor":  left_distance,
        "right_sensor":  right_distance,
        # "direction": robot_modes.direction,
        # "wheel_speed": robot_modes.wheel_speed
    }
    return jsonify(status)

@app.route('/video_feed/<source>')
def video_feed(source):
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
