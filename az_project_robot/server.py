from src.robot.modes import Modes
from flask import Flask, render_template, Response, jsonify, request
import threading
from src.utils.control_utils import getch, direction, set_motors_direction
import time
import cv2
from datetime import datetime

app = Flask(__name__)
robot = Modes()
# Khởi tạo tài nguyên cần thiết để cho color, object detection
#stop_event = robot.stop_search_event
frame_q = robot.frame_queue
#time.sleep(1)

def gen_frames(mode):   
    while True:
        if not frame_q.empty():
            frame_data = frame_q.get()
            if mode == "color":
                output_frame = frame_data[1]  # Assuming frame_data[0] is the color frame
            elif mode == "object":
                output_frame = frame_data[18]  # Assuming frame_data[1] is the object frame
            else:
                output_frame = None
            if output_frame is not None:
                _, buffer = cv2.imencode('.jpg', output_frame)
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

# Route: Luồng video
@app.route('/video_feed/color')
def video_feed_color_detection():
    return Response(gen_frames("color"), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route: Video feed for object detection
@app.route('/video_feed/object')
def video_feed_object_detection():
    return Response(gen_frames("object"), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route: Lệnh chế độ tự động
@app.route('/auto-command', methods=['POST'])
def auto_command():
    data = request.get_json()
    robot.daily_mission = int(data.get("value1", 0))
    start_time = data.get("startTime")  # Nhận thời gian bắt đầu
    end_time = data.get("endTime")      # Nhận thời gian kết thúc
    
    # Cập nhật thời gian hoạt động cho robot
    robot.OPERATION_START_TIME = datetime.strptime(start_time, '%H:%M').time()
    robot.OPERATION_END_TIME = datetime.strptime(end_time, '%H:%M').time()
    
    # Chuyển sang chế độ tự động
    robot.switch_mode("automatic")
    
    # Start color and object detection threads
    # robot.start_video_stream()  # Khởi động luồng video
    # robot.start_color_detection()
    # robot.start_object_detection()
    robot.automatic_mode()
    return jsonify({"success": True})

# Route: Điều khiển chế độ (auto/manual)
@app.route('/mode', methods=['POST'])
def mode():
    data = request.get_json()
    mode = data.get("mode")
    if mode == "manual":
        robot.switch_mode("manual")
        #threading.Thread(target=robot.manual_control).start()
    elif mode == "auto":
        robot.switch_mode("automatic")
        # Không khởi động tự động ngay tại đây
    return jsonify({"success": True})

@app.route('/status', methods=['GET'])
def status():
    front = robot.ultrasonic_sensors.get_distance("front")
    left = robot.ultrasonic_sensors.get_distance("left")
    right = robot.ultrasonic_sensors.get_distance("right")
    battery_status = robot.battery.read_battery_status()
    servo_down = robot.bottom_angle
    servo_up = robot.top_angle
    
    return jsonify({
        "voltage": battery_status[0],
        "current": battery_status[1],
        "power": battery_status[2],
        "battery": battery_status[3],
        "remaining_time": battery_status[4],
        "water": "Có nước" if robot.is_watering else "Hết nước",
        "wheel_speed": robot.vx,
        "front_sensor": front,
        "left_sensor": left,
        "right_sensor": right,
        "robot_direction": robot.direction,
        "duty": robot.duty,  # Trạng thái hiện tại của robot
        "state": robot.state,  # Hành động hiện tại của robot
        "servo_down": servo_down,
        "servo_up": servo_up
    })

# Route: Điều khiển robot (manual)
@app.route('/control', methods=['POST'])
def manual_control_api():
    data = request.json
    command = data.get('command', None)
    if not command:
        return jsonify({"error": "No command provided"}), 400
    if command in ['+', '=', '-', '_', 'w', 'a', 's', 'd', 'q', 'e', 'z', 'x', 'r', 'p', '1', '2', '3']: 
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
        elif command in direction:
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
