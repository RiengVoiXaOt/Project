from flask import Flask, render_template, request, Response, jsonify
# import cv2
# from src.robot.modes import Modes
# from src.utils.control_utils import set_motors_direction
from src.hardware.water_sensor import WaterLevelSensor
# from src.hardware.battery import BatteryMonitor
# from src.hardware.ultrasonic import UltrasonicSensors
# from src.vision.color_detection import color_detection_loop
# from src.vision.object_detection import object_detection_loop
# from src.vision.video_stream import VideoStream
# from queue import Queue
# from threading import Thread, Event
# import time
app = Flask(__name__)

# Khởi tạo một số thứ cần thiết bên phân status 
#robot_modes = Modes()  # Khởi tạo đối tượng quản lý robot
water_level = WaterLevelSensor()
#battery_level = BatteryMonitor()
#ultrasonic_sensor = UltrasonicSensors()
# Khởi tạo tài nguyên cần thiết để cho color,object detection
# stop_event = Event()
# frame_queue = Queue(maxsize=1)
# videostream = VideoStream(resolution=(640, 480), framerate=30).start()
# time.sleep(1)  

# color_thread = Thread(target=color_detection_loop, args=(videostream, 320, 240, 1000, stop_event, frame_queue, 10), daemon=True)
# object_thread = Thread(target=object_detection_loop, args=(videostream, stop_event, frame_queue), daemon=True)
# def gen_frames(mode):
#     while True:
#         if not frame_queue.empty():
#             frame, frame2 = frame_queue.get()
#             if mode == "color":
#                 output_frame = frame  # Luồng Color Detection
#             elif mode == "object":
#                 output_frame = frame2  # Luồng Object Detection
#             else:
#                 output_frame = None
#             if output_frame is not None:
#                 _, buffer = cv2.imencode('.jpg', output_frame)
#                 yield (b'--frame\r\n'
#                        b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
@app.route('/')
def index():
    return render_template('index.html')  

# @app.route('/manual_control', methods=['POST'])
# def manual_control():
#     data = request.json
#     command = data.get('command', None)

#     if not command:
#         return jsonify({"error": "No command provided"}), 400

#     response = robot_modes.execute_command(command)
#     set_motors_direction(command, robot_modes.vx, robot_modes.vy, robot_modes.theta)
    
#     return jsonify(response)

@app.route('/status')
def get_status():
    #voltage, current, power, battery_percentage, remaining_time_hours = battery_level.read_battery_status()
    # front_distance = ultrasonic_sensor.get_distance("front")
    # left_distance = ultrasonic_sensor.get_distance("left")
    # right_distance = ultrasonic_sensor.get_distance("right")
    water_present = water_level.is_water_present()
    
    # Tạo dictionary status với các thông tin khác
    status = {
        # "voltage": voltage,
        # "current": current,
        # "power": power,
        # "percentage": battery_percentage,
        # "remaining_time": remaining_time_hours,

        "water": "Có nước" if water_present else "Khát quá",
        # "front_sensor": front_distance,
        # "left_sensor":  left_distance,
        # "right_sensor":  right_distance,
        # "direction": robot_modes.direction,
        # "wheel_speed": robot_modes.wheel_speed
    }
    return jsonify(status)

@app.route('/video_feed/color')
def video_feed_color_detection():
    return Response(gen_frames("color"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed/object')
def video_feed_object_detection():
    return Response(gen_frames("object"), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    #try:
        # Chạy các luồng xử lý song song
        # color_thread.start()
        # object_thread.start()        # Chạy server Flask
        app.run(host='0.0.0.0', port=5000)
    #except KeyboardInterrupt:
        # stop_event.set()
        # videostream.stop()