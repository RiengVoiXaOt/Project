from src.robot.modes import Modes
from flask import Flask, render_template, Response, jsonify, request
import threading

app = Flask(__name__)
robot = Modes()

# Route: Trang điều khiển chính
@app.route('/')
def index():
    return render_template('index.html')

# Route: Luồng video
@app.route('/video_feed/<string:camera>')
def video_feed(camera):
    if camera == "color":
        return Response(generate_video(robot.videostream.start_color_detection), mimetype='multipart/x-mixed-replace; boundary=frame')
    elif camera == "object":
        return Response(generate_video(robot.videostream.start_object_detection), mimetype='multipart/x-mixed-replace; boundary=frame')
    else:
        return "Invalid camera source", 400

def generate_video(stream):
    while True:
        frame = stream.read()
        if frame is not None:
            _, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        else:
            break

# Route: Cập nhật trạng thái robot
@app.route('/status', methods=['GET'])
def status():
    sensor_data = robot.get_sensor_data()
    battery_status = robot.battery.read_battery_status()
    return jsonify({
        "voltage": battery_status[0],
        "current": battery_status[1],
        "power": battery_status[2],
        "battery": battery_status[3],
        "remaining_time": battery_status[4],
        "water": "Có nước" if robot.is_watering else "Hết nước ",
        "wheel_speed": robot.vx,
        "front_sensor": sensor_data["front"],
        "left_sensor": sensor_data["left"],
        "right_sensor": sensor_data["right"],
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
def control():
    data = request.get_json()
    command = data.get("command")
    robot.manual_control()
    return jsonify({"success": True})

# Route: Lệnh chế độ tự động
@app.route('/auto-command', methods=['POST'])
def auto_command():
    data = request.get_json()
    robot.number_of_plant = int(data.get("value1", 0))
    # Xử lý thời gian nếu cần từ `value2`
    return jsonify({"success": True})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
