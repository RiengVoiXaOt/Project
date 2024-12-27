from flask import Flask, render_template, Response, jsonify, request
from threading import Thread
from src.hardware.relay import RelayControl
from src.hardware.motors import Motors
from src.hardware.ultrasonic import UltrasonicSensors
from src.vision.video_stream import VideoStream
from src.vision.color_detection import color_detection_loop
from src.vision.object_detection import object_detection_loop
from src.utils.control_utils import set_motors_direction
from src.hardware.servos import ServoControl
from src.hardware.battery import BatteryMonitor
from src.robot.modes import Modes
import time

app = Flask(__name__)
robot = Modes()

# Khởi động luồng video và các luồng phát hiện đối tượng và màu
def start_detection_threads():
    robot.start_video_stream()
    Thread(target=robot.start_color_detection, daemon=True).start()
    Thread(target=robot.start_object_detection, daemon=True).start()

# Route: Trang chính
@app.route('/')
def index():
    return render_template('index.html')

# Route: Video feed cho phát hiện màu
@app.route('/video_feed/color')
def video_feed_color_detection():
    return Response(robot.videostream.get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route: Video feed cho phát hiện đối tượng
@app.route('/video_feed/object')
def video_feed_object_detection():
    return Response(robot.videostream.get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route: Cập nhật trạng thái robot
@app.route('/status', methods=['GET'])
def status():
    front_distance = robot.ultrasonic_sensors.get_distance("front")
    left_distance = robot.ultrasonic_sensors.get_distance("left")
    right_distance = robot.ultrasonic_sensors.get_distance("right")
    battery_status = robot.battery.read_battery_status()
    
    return jsonify({
        "voltage": battery_status[0],
        "current": battery_status[1],
        "power": battery_status[2],
        "battery": battery_status[3],
        "remaining_time": battery_status[4],
        "water": "Có nước" if robot.is_watering else "Hết nước",
        "wheel_speed": robot.vx,
        "front_sensor": front_distance,
        "left_sensor": left_distance,
        "right_sensor": right_distance,
        "robot_direction": robot.direction,
        "servo_down": robot.bottom_angle,
        "servo_up": robot.top_angle
    })

# Route: Chuyển đổi chế độ
@app.route('/mode', methods=['POST'])
def mode():
    data = request.get_json()
    mode = data.get("mode")
    if mode == "auto":
        robot.switch_mode("automatic")
        start_detection_threads()  # Khởi động các luồng phát hiện
        return jsonify({"success": True})
    elif mode == "manual":
        robot.switch_mode("manual")
        return jsonify({"success": True})
    return jsonify({"error": "Invalid mode"}), 400

# Route: Lệnh chế độ tự động
@app.route('/auto-command', methods=['POST'])
def auto_command():
    data = request.get_json()
    robot.number_of_plant = int(data.get("value1", 0))
    start_time = data.get("startTime")
    end_time = data.get("endTime")
    
    # Cập nhật thời gian hoạt động cho robot
    robot.OPERATION_START_TIME = time.strptime(start_time, '%H:%M').tm_hour
    robot.OPERATION_END_TIME = time.strptime(end_time, '%H:%M').tm_hour

    # Bắt đầu chế độ tự động
    Thread(target=robot.automatic_mode, daemon=True).start()  # Chạy hàm automatic_mode trong luồng mới
    return jsonify({"success": True})

# Route: Điều khiển robot
@app.route('/control', methods=['POST'])
def manual_control_api():
    data = request.json
    command = data.get('command', None)
    if not command:
        return jsonify({"error": "No command provided"}), 400
    if command in ['+', '=', '-', '_', 'w', 'a', 's', 'd', 'q', 'e', 'z', 'x', 'r','p','1','2','3']: #, '7', '8', '9', '0'
        if command in ['+', '=']:
            robot.n = min(robot.n + 1, 10)
            robot.vx = robot.n * robot.speed
            robot.vy = robot.n * robot.speed
            robot.update_state(f"speed increased to {robot.n * 10}%")
        elif command in ['-', '_']:
            robot.n = max(robot.n - 1, 0)
            robot.vx = robot.n * robot.speed
            robot.vy = robot.n * robot.speed
            robot.update_state(f"speed decreased to {robot.n * 10}%")
        elif command in direction :
            current_direction = direction[command]
            robot.set_motors_direction(current_direction, robot.vx, robot.vy, robot.theta)
            robot.update_state(f"moving {current_direction}")
        elif command == 'r':
            robot.relay_control.run_relay_for_duration()
            robot.update_state("watering activated")
        return jsonify({"status": "Command executed successfully", "current_state": command})
    return jsonify({"error": "Invalid command"}), 400


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
