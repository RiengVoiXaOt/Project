from src.robot.modes import Modes
from flask import Flask, render_template, Response, jsonify, request
import threading
from src.utils.control_utils import getch, direction, set_motors_direction
import time
import cv2
app = Flask(__name__)
robot = Modes()
# # Khởi tạo tài nguyên cần thiết để cho color,object detection
stop_event = robot.stop_search_event
frame_q = robot.frame_queue
videostream = robot.videostream
time.sleep(1)  

print("-----------------------")
print(frame_q)
print("-----------------------")


object_thread = robot.start_object_detection()
color_thread = robot.start_color_detection()
def gen_frames(mode):
    while True:
        if not frame_queue.empty():
            frame, frame2 = frame_q.get()
            if mode == "color":
                output_frame = frame  # Luồng Color Detection
            elif mode == "object":
                output_frame = frame2  # Luồng Object Detection
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

@app.route('/video_feed/object')
def video_feed_object_detection():
    return Response(gen_frames("object"), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route: Cập nhật trạng thái robot
@app.route('/status', methods=['GET'])
def status():
    front = robot.ultrasonic_sensors.get_distance("front")
    left = robot.ultrasonic_sensors.get_distance( "left")
    right = robot.ultrasonic_sensors.get_distance("right")

    battery_status = robot.battery.read_battery_status()
    return jsonify({
        "voltage": battery_status[0],
        "current": battery_status[1],
        "power": battery_status[2],
        "battery": battery_status[3],
        "remaining_time": battery_status[4],
        "water": "Có nước" if robot.is_watering else "Hết nước ",
        "wheel_speed": robot.vx,
        "front_sensor": front,
        "left_sensor": left,
        "right_sensor": right,
        "robot_direction": robot.direction
    })

# Route: Điều khiển chế độ (auto/manual)
@app.route('/mode', methods=['POST'])
def mode():
    data = request.get_json()
    mode = data.get("mode")
    if mode == "manual":
        robot.switch_mode("manual")
        threading.Thread(target=robot.manual_control).start()
    elif mode == "auto":
        robot.switch_mode("automatic")
        robot.start()
    return jsonify({"success": True})

# Route: Điều khiển robot (manual)
@app.route('/control', methods=['POST'])
def manual_control_api():
    data = request.json
    command = data.get('command', None)
    if not command:
        return jsonify({"error": "No command provided"}), 400

    if command in ['+', '=', '-', '_', 'w', 'a', 's', 'd', 'q', 'e', 'z', 'x', 'r','p', '7', '8', '9', '0']:
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
        elif command in direction :#['w', 'a', 's', 'd', 'q', 'e', 'z', 'x']:
            current_direction = direction[command]
            robot.set_motors_direction(current_direction, robot.vx, robot.vy, robot.theta)
            robot.update_state(f"moving {current_direction}")
        elif command == 'r':
            robot.relay_control.run_relay_for_duration()
            robot.update_state("watering activated")
        elif command == 'p':
            robot.top_servo.reset()
            robot.bottom_servo.reset()
            robot.update_state("manual control exited")
            robot.set_motors_direction('stop', 0, 0, 0)
        elif command in ['7', '8', '9', '0']:
            robot.control_servos(command)
        return jsonify({"status": "Command executed successfully", "current_state": state})

    return jsonify({"error": "Invalid command"}), 400

# Route: Lệnh chế độ tự động
@app.route('/auto-command', methods=['POST'])
def auto_command():
    data = request.get_json()
    robot.number_of_plant = int(data.get("value1", 0))
    # Xử lý thời gian nếu cần từ `value2`
    return jsonify({"success": True})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
