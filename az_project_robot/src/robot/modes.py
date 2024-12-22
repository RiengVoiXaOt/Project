from time import sleep, time
from src.hardware.relay import RelayControl
from src.hardware.motors import Motors
from src.hardware.ultrasonic import UltrasonicSensors
from src.vision.color_detection import color_detection_loop
from src.vision.object_detection import object_detection_loop
from src.utils.control_utils import getch, direction, set_motors_direction
from src.hardware.servos import ServoControl
from src.utils.image_utils import load_labels, load_model, get_model_details, draw_detections, preprocess_frame, calculate_fps, analyze_detection
from src.vision.video_stream import VideoStream
from src.hardware.battery import BatteryMonitor
from threading import Thread, Event, Lock
from datetime import datetime
from queue import Queue
import numpy as np
import warnings
import os
import cv2

"""
Robot sẽ có hai chế độ hoạt động chính:
a. Chế Độ Tự Động
    Mô tả: Đây là chế độ mặc định, nơi robot tự động thực hiện các nhiệm vụ mà không cần sự can thiệp của người dùng.
    Chức năng:
    Tưới cây: Robot sẽ di chuyển đến vị trí các cây cần tưới và thực hiện tưới.
    Tránh chướng ngại vật: Sử dụng cảm biến siêu âm để phát hiện và tránh các vật cản.
    Nhận diện đường line: Sử dụng thuật toán color detection để không đi qua đường line màu đen.
    Tìm kiếm vật: Sử dụng object detection để phát hiện và tương tác với các vật thể.
    Quay về trạm sạc: Sau khi hoàn thành nhiệm vụ tưới cây, robot sẽ tìm đường về trạm sạc bằng cách nhận diện đường line màu đỏ.
b. Chế Độ Thủ Công
    Mô tả: Người dùng có thể điều khiển robot trực tiếp thông qua bàn phím.
    Chức năng:
    Điều khiển hướng và tốc độ di chuyển của robot.
    Bật/tắt relay để tưới cây.
    Điều khiển góc quay của servo (servo trên và servo dưới).
    Chuyển đổi giữa chế độ tự động và chế độ thủ công.
"""

class Modes:
    SAFE_DISTANCE = 17  # Khoảng cách an toàn (cm)
    CRITICAL_DISTANCE = 15  # Ngưỡng cảnh báo (cm)
    MAX_HISTORY = 10  # Giới hạn lịch sử khoảng cách
    MAX_ANGLE = 120
    MIN_ANGLE = 0
    DEFAULT_ANGLE_TOP = 90
    DEFAULT_ANGLE_BOTTOM = 60
    def __init__(self, n=None, theta=None):
        # Initialize hardware components
        self.relay_control = RelayControl(5)
        self.top_servo = ServoControl(channel=0)
        self.bottom_servo = ServoControl(channel=1)
        self.ultrasonic_sensors = UltrasonicSensors()
        self.motors = Motors()
        self.battery = BatteryMonitor()

        # Operational variables
        self.manual_mode = False
        self.is_watering = False
        self.charge_station_found = False
        self.check_event = Event()
        self.stop_event = Event()
        self.frame_queue = Queue(maxsize=1)
        self.videostream = VideoStream(resolution=(640, 480), framerate=30)

        # Default settings
        self.n = n if n is not None else 3
        self.theta = theta if theta is not None else 0
        self.speed = 0.04309596457
        self.vx = self.n * self.speed
        self.vy = self.n * self.speed
        self.state = "stopped"
        self.active_mode = "automatic"
        self.distance_history = {
            "front": [], "left": [], "right": [], "front_left": [], "front_right": []
        }
        self.last_activity_time = time()
        self.is_running = False
        self.OPERATION_START_TIME = datetime.now().replace(hour=12, minute=40,second=0)
        
    def switch_mode(self):
        if self.active_mode == 'manual':
            self.active_mode = 'automatic'
        else:
            self.active_mode = 'manual'
    def start(self):
        self.check_event.clear()
        Thread(target=self.automatic_mode).start()
################################ Các hàm liên quan đến điều khiển thủ công ################################
    def manual_control(self):
        while self.manual_mode:
            print("Enter command (w/a/s/d/q/e/z/x/1/2/3 to move, r to toggle relay, p to quit): ")
            command = getch()
            if command == 'p':
                self.top_servo.reset()
                self.bottom_servo.reset()
                self.update_state("manual control exited")
                set_motors_direction('stop', 0, 0, 0)
                print("Exiting manual control.")
                return
            elif command in ['+', '=']:
                self.n = min(self.n + 1, 10)
                self.vx = self.n * self.speed
                self.vy = self.n * self.speed
                self.update_state(f"speed increased to {self.n * 10}%")
                print(f"Tốc độ tăng lên {self.n * 10}%")
            elif command in ['-', '_']:
                self.n = max(self.n - 1, 0)
                self.vx = self.n * self.speed
                self.vy = self.n * self.speed
                self.update_state(f"speed decreased to {self.n * 10}%")
                print(f"Tốc độ giảm xuống {self.n * 10}%")
            elif command in direction:
                current_direction = direction[command]
                print("Direction: " + current_direction)
                set_motors_direction(current_direction, self.vx, self.vy, self.theta)
                self.update_state(f"moving {current_direction}")
            elif command == 'r':
                self.relay_control.run_relay_for_duration()
                self.update_state("watering activated")
            elif command in ['7', '8', '9', '0']:
                self.control_servos(command)
            else:
                self.update_state("invalid command")
                print("Lệnh không hợp lệ. Vui lòng thử lại.")

    def control_servos(self, command):
        if command == '7':
            self.bottom_servo.move_up()
            self.update_state("bottom servo moved up")
        elif command == '8':
            self.bottom_servo.move_down()
            self.update_state("bottom servo moved down")
        elif command == '9':
            self.top_servo.move_up()
            self.update_state("top servo moved up")
        elif command == '0':
            self.top_servo.move_down()
            self.update_state("top servo moved down")
            
################################ Các hàm liên quan đến chạy tránh vật cản ################################

    def update_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            print(f"Trạng thái hiện tại: {self.state}")

    def move_forward(self):
        set_motors_direction("go_forward", self.vx, self.vy, 0)
        self.update_state("moving forward")

    def move_backward(self):
        set_motors_direction("go_backward", self.vx, self.vy, 0)
        self.update_state("moving backward")

    def stop_robot(self):
        set_motors_direction("stop", self.vx, self.vy, 0)
        self.update_state("stopped")

    def rotate_robot(self, direction, angle=90):
        max_rotate_time = angle / 90 * 3
        rotate_start_time = time()

        while time() - rotate_start_time < max_rotate_time:
            front_distance = self.ultrasonic_sensors.get_distance("front")
            if front_distance >= 1.5 * self.SAFE_DISTANCE:
                self.stop_robot()
                break
            set_motors_direction(direction, self.vx, self.vy, 0)
            sleep(0.1)
        self.stop_robot()
        self.update_state(f"rotating {'right' if direction == 'rotate_right' else 'left'}")

    def avoid_obstacles(self):
        """Tránh chướng ngại vật bằng cách sử dụng cảm biến siêu âm."""
        front_distance = self.ultrasonic_sensors.get_distance("front")
        left_distance = self.ultrasonic_sensors.get_distance("left")
        right_distance = self.ultrasonic_sensors.get_distance("right")
        front_left_distance = self.ultrasonic_sensors.get_distance("front_left")
        front_right_distance = self.ultrasonic_sensors.get_distance("front_right")

        if None in (front_distance, left_distance, right_distance,front_left_distance,front_right_distance):
            print("Lỗi cảm biến siêu âm: Không nhận được dữ liệu.")
            self.stop_robot()
            return

        self.update_distance_history(front_distance, left_distance, right_distance,front_left_distance,front_right_distance)

        f_state = front_distance < self.SAFE_DISTANCE
        l_state = left_distance < self.SAFE_DISTANCE
        r_state = right_distance < self.SAFE_DISTANCE
        f_l_state = front_left_distance < self.SAFE_DISTANCE
        f_r_state = front_right_distance < self.SAFE_DISTANCE
        if not f_state:
            if not r_state and not l_state and not f_l_state and not f_r_state:
                self.move_forward()
            if r_state and right_distance < self.CRITICAL_DISTANCE:
                set_motors_direction('go_left', self.vx, self.vy, 0)
                self.update_state("going left")
            elif l_state and left_distance < self.CRITICAL_DISTANCE:
                set_motors_direction('go_right', self.vx, self.vy, 0)
                self.update_state("going right")
            elif f_l_state and front_left_distance < self.SAFE_DISTANCE:
                set_motors_direction('rotate_right', self.vx, self.vy, 0)
                self.update_state("rotate_right")
            elif f_r_state and front_right_distance < self.SAFE_DISTANCE :
                set_motors_direction('rotate_left', self.vx, self.vy, 0)
                self.update_state("rotate_left")
        else:
            if front_distance <= self.CRITICAL_DISTANCE:
                self.move_backward()
                return
            self.handle_side_obstacles(l_state, r_state, left_distance, right_distance)

    def update_distance_history(self, front, left, right, front_left,front_right):
        """Lưu lịch sử khoảng cách và giới hạn kích thước."""
        for key, value in zip(['front', 'left', 'right','front_left','front_right'], [front, left, right,front_left,front_right]):
            self.distance_history[key].append(value)
            if len(self.distance_history[key]) > self.MAX_HISTORY:
                self.distance_history[key].pop(0)

    def handle_side_obstacles(self, l_state, r_state, left_distance, right_distance):
        """Xử lý vật cản ở hai bên robot."""
        if l_state and r_state:
            if right_distance > left_distance:
                self.rotate_robot('rotate_right')
            else:
                self.rotate_robot('rotate_left')
        elif l_state and not r_state:
            self.rotate_robot('rotate_right')
        elif not l_state and r_state:
            self.rotate_robot('rotate_left')
        elif not l_state and not r_state:
            self.rotate_robot('rotate_right')
            
################################ Các hàm liên quan đến luồng thị giác máy tính ################################
    def start_object_detection(self):
        """Bắt đầu luồng phát hiện đối tượng."""
        self.stop_event.clear()
        object_thread = Thread(target=object_detection_loop, args=(self.videostream, self.stop_event, self.frame_queue), daemon=True)
        object_thread.start()
        return object_thread
        
    def start_color_detection(self):
        """Bắt đầu luồng phát hiện màu sắc."""
        self.stop_event.clear()
        color_thread = Thread(target=color_detection_loop, args=(self.videostream, 320, 240, 1000, self.stop_event, self.frame_queue, 10), daemon=True)
        color_thread.start()
        return color_thread
    
    def process_frame(self):
        """Xử lý khung hình từ hàng đợi."""
        return self.frame_queue.get()
    
    def start_video_stream(self):
        """Khởi động VideoStream."""
        self.videostream.start()
        sleep(1)
        
    ################################ Các hàm liên quan đến điều khiển tự động ################################
    def automatic_mode(self):
        self.start_video_stream()  # Khởi động luồng video
        color_thread = self.start_color_detection()  # Bắt đầu luồng nhận diện màu
        object_thread = self.start_object_detection()  # Bắt đầu luồng nhận diện đối tượng
        try:
            while not self.stop_event.is_set():  # Vòng lặp chạy khi không dừng và không ở chế độ thủ công
                if not self.manual_mode:
                    """Chế độ tự động cho robot thực hiện nhiệm vụ."""
                    print("Chế độ tự động đang chạy...")
                    if not frame_queue.empty():
                    (frame_color, mask_red, mask_black, status_red, deviation_x_red, deviation_y_red,
                    status_black, deviation_x_black, deviation_y_black, contours_black, contours_red,
                    status_water, status_charger, deviation_x_water, deviation_y_water,
                    deviation_x_charger, deviation_y_charger, frame_object) = process_frame(frame_queue)
                    self.avoid_obstacles()  # Tránh chướng ngại vật
                    # Gọi các hàm khác để thực hiện nhiệm vụ như tưới cây, tìm kiếm vật thể, v.v.
                    
                    # Kiểm tra pin và quay về trạm sạc nếu cần
                    # if self.should_return_to_charge_station():
                    #     print("Pin yếu, đang quay về trạm sạc...")
                    #     self.return_to_charge_station()
                    #     continue
                    
                    # Thêm một khoảng thời gian nghỉ để giảm tải CPU
                    sleep(0.05)  # Giúp vòng lặp không chiếm quá nhiều tài nguyên
                else:
                    print("Chuyển sang chế độ thủ công. Thoát khỏi tự động.")
                    self.update_state("Automatic exited")
                    break  # Thoát khỏi vòng lặp nếu chuyển sang chế độ thủ công

                # Thêm log để theo dõi trạng thái
                print(f"Manual Mode: {self.manual_mode}, Stop Event: {self.stop_event.is_set()}")

            sleep(0.1)  # Thêm thời gian nghỉ để giảm tải CPU
        except Exception as e:
            print(f"Error in automatic mode: {e}")
            
        finally:
            self.stop_event.set()  # Đặt cờ dừng
            if 'videostream' in locals():
                self.videostream.stop()  # Dừng video stream
            cv2.destroyAllWindows()  # Đóng tất cả cửa sổ OpenCV
            
    def check_battery_and_time(self, now):
        """Kiểm tra trạng thái pin và thời gian hoạt động."""
        if now >= self.OPERATION_START_TIME:
            print(f"Đã đến giờ hoạt động. Bắt đầu vào lúc {self.OPERATION_START_TIME}.")
            return self.battery.read_battery_status()[3] > 25  # Giả sử đây là cách lấy trạng thái pin
        return False
    
