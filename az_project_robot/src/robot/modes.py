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
    SAFE_DISTANCE = 30  # Khoảng cách an toàn (cm)
    CRITICAL_DISTANCE = 20  # Ngưỡng cảnh báo (cm)
    MAX_HISTORY = 15  # Giới hạn lịch sử khoảng cách    
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
        self.is_tracking_charger = False
        self.is_tracking_warter = False
        self.is_docking = False
        self.is_resting = False  # Biến trạng thái nghỉ ngơi
        self.manual_mode = False
        self.is_watering = False
        self.charge_station_found = False
        self.daily_mission = 3  # Số lượng cây cần tưới mỗi ngày
        self.current_mission_count = 0  # Đếm số cây đã tưới trong ngày
        self.mission = True  # Khởi tạo biến mission
        self.is_running = False
        self.avoid_yellow_line = False
        self.red_folowing = False
        self.search_object = False
        self.firtstart = True
        self.watered = False
        self.finding_angle = True
        self.status_charger_history = []  # Mảng để lưu trữ trạng thái
        self.status_water_history = []  # Mảng để lưu trữ trạng thái
        self.reset_threshold = 7  # Số lượng trạng thái cần kiểm tra
        self.number_of_plant = 2
        
        self.check_event = Event()
        self.stop_event = Event()
        self.frame_queue = Queue(maxsize=1)
        self.videostream = VideoStream(resolution=(640, 480), framerate=30)
        self.result_queue = Queue(maxsize=3)  # Hàng đợi để lưu kết quả từ luồng quét
        self.stop_search_event = Event()
        self.search_thread = None

        # Default settings
        self.n = n if n is not None else 2
        self.theta = theta if theta is not None else 0
        self.speed = 0.04309596457
        self.vx = self.n * self.speed
        self.vy = self.n * self.speed
        self.state = "start"
        self.direction = "stoped"
        self.active_mode = "automatic"
        self.current_state = "None"
        self.distance_history = { "front": [], "left": [], "right": [], "front_left": [], "front_right": []}
        self.servo_angle_history_bottom = []
        self.servo_angle_history_top = []
        self.last_activity_time = time()
        self.OPERATION_START_TIME = datetime.now().replace(hour=12, minute=0,second=0)
        self.OPERATION_END_TIME = datetime.now().replace(hour=13, minute=0,second=0)
        self.last_detection_time = time()
        self.set_motors_direction = set_motors_direction
        
        # Đưa servo về góc mặc định
        self.bottom_angle = self.DEFAULT_ANGLE_BOTTOM  # Góc ban đầu của servo dưới
        self.top_angle = self.DEFAULT_ANGLE_TOP  # Góc ban đầu của servo trên
        self.bottom_servo.move_to_angle(self.DEFAULT_ANGLE_BOTTOM)
        self.top_servo.move_to_angle(self.DEFAULT_ANGLE_TOP)
        
    def switch_mode(self, activate_mode):
        if activate_mode == 'automatic':
            self.active_mode = 'automatic'
            self.manual_mode = False
        else:
            self.active_mode = 'manual'
            self.manual_mode = True
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
                self.set_motors_direction('stop', 0, 0, 0)
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
                self.set_motors_direction(current_direction, self.vx, self.vy, self.theta)
                self.update_state(f"moving {current_direction}")
            elif command == 'r':
                self.relay_control.run_relay_for_duration()
                self.update_state("watering activated")
            elif command in ['7', '8', '9', '0']:
                self.control_servos(command)
            else:
                self.update_state("invalid command")
                print("Lệnh không hợp lệ. Vui lòng thử lại.")
    def execute_command(self, command):
        if command == 'w':
            self.set_motors_direction('forward', self.vx, self.vy, self.theta)
        elif command == 's':
            self.set_motors_direction('backward', self.vx, self.vy, self.theta)
        elif command == 'n':
            self.set_motors_direction('stop', self.vx, self.vy, self.theta)
        elif command == 'a':
            self.set_motors_direction('left', self.vx, self.vy, self.theta)
        elif command == 'd':
            self.set_motors_direction('right', self.vx, self.vy, self.theta)
        elif command == 'r':
            self.relay_control.run_relay_for_duration()  # Kích hoạt tưới nước
        elif command == '+':
            self.n = min(self.n + 1, 15)  # Tăng tốc độ
            self.vx = self.n * self.speed
            self.vy = self.n * self.speed
        elif command == '-':
            self.n = max(self.n - 1, 0)  # Giảm tốc độ
            self.vx = self.n * self.speed
            self.vy = self.n * self.speed
        else:
            print("Lệnh không hợp lệ:", command)
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
            print(f"State transition: {self.state} -> {new_state}")
            print(f"Trạng thái hiện tại: {self.state}")
    def update_direction(self, new_direction):
        if self.direction != new_direction:
            self.direction = new_direction
            print(f"Trạng thái hiện tại: {self.direction}")

    def move_forward(self):
        self.set_motors_direction("go_forward", self.vx, self.vy, 0)
        self.update_state("moving forward")

    def move_backward(self):
        self.set_motors_direction("go_backward", self.vx, self.vy, 0)
        self.update_state("moving backward")

    def stop_robot(self):
        self.set_motors_direction("stop", self.vx, self.vy, 0)
        self.update_state("stopped")

    def rotate_robot(self, direction, angle=90):
        self.update_state("Tránh vật cản")
        max_rotate_time = angle / 90 * 3
        rotate_start_time = time()

        while time() - rotate_start_time < max_rotate_time:
            front_distance = self.ultrasonic_sensors.get_distance("front")
            if front_distance >= 1.5 * self.SAFE_DISTANCE:
                self.stop_robot()
                break
            self.set_motors_direction(direction, self.vx, self.vy, 0)
            sleep(0.1)
        self.stop_robot()
        self.update_direction(f"rotating {'right' if direction == 'rotate_right' else 'left'}")

    def avoid_and_navigate(self, front_distance, left_distance, right_distance,front_left_distance, front_right_distance):

        if None in (front_distance, left_distance, right_distance,front_left_distance,front_right_distance):
            self.update_state("Lỗi cảm biến siêu âm: Không nhận được dữ liệu.")
            self.stop_robot()
            return

        self.update_distance_history(front_distance, left_distance, right_distance,front_left_distance,front_right_distance)

        f_state = front_distance < self.SAFE_DISTANCE
        l_state = left_distance < self.SAFE_DISTANCE
        r_state = right_distance < self.SAFE_DISTANCE
        f_l_state = front_left_distance < self.SAFE_DISTANCE
        f_r_state = front_right_distance < self.SAFE_DISTANCE 
        
        if not f_state:
            if left_distance > self.CRITICAL_DISTANCE and right_distance > self.CRITICAL_DISTANCE and not f_l_state and not f_r_state:
                self.move_forward()
                self.update_direction("Đang đi thẳng")
            if right_distance <= self.CRITICAL_DISTANCE:
                self.set_motors_direction('go_left', self.vx, self.vy, 0)
                self.update_direction("going left")
            elif left_distance <= self.CRITICAL_DISTANCE:
                self.set_motors_direction('go_right', self.vx, self.vy, 0)
                self.update_direction("going right")
            elif front_left_distance <= self.SAFE_DISTANCE:
                self.set_motors_direction('rotate_right', self.vx, self.vy, 0)
                self.update_direction("rotate_right")
            elif front_right_distance <= self.SAFE_DISTANCE:
                self.set_motors_direction('rotate_left', self.vx, self.vy, 0)
                self.update_direction("rotate_left")
        else:
            if front_distance <= self.CRITICAL_DISTANCE or front_left_distance <= self.CRITICAL_DISTANCE or front_right_distance <= self.CRITICAL_DISTANCE:
                self.move_backward() 
                self.update_direction("Đi lùi")
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
                self.update_direction("Xoay phải")
            else:
                self.rotate_robot('rotate_left')
                self.update_direction("Xoay trái")
        elif l_state and not r_state:
            self.rotate_robot('rotate_right')
            self.update_direction("Xoay phải")
        elif not l_state and r_state:
            self.rotate_robot('rotate_left')
            self.update_direction("Xoay trái")
        elif not l_state and not r_state:
            self.rotate_robot('rotate_right')
            self.update_direction("Xoay phải")
            
################################ Các hàm liên quan đến luồng thị giác máy tính ################################
    def start_object_detection(self):
        """Bắt đầu luồng phát hiện đối tượng."""
        self.stop_event.clear()
        object_thread = Thread(target=object_detection_loop, args=(self.videostream, self.stop_event, self.frame_queue), daemon=True)
        object_thread.start()
        self.update_state("Bắt đầu luồng phát hiện đối tượng")
        return object_thread
        
    def start_color_detection(self):
        """Bắt đầu luồng phát hiện màu sắc."""
        self.stop_event.clear()
        color_thread = Thread(target=color_detection_loop, args=(self.videostream, 320, 240, 1000, self.stop_event, self.frame_queue, 15), daemon=True)
        color_thread.start()
        self.update_state("Bắt đầu luồng phát hiện màu sắc")
        return color_thread
    
    def process_frame(self):
        """Xử lý khung hình từ hàng đợi."""
        return self.frame_queue.get()
    
    def start_video_stream(self):
        """Khởi động VideoStream."""
        self.videostream.start()
        sleep(1)
    def process_frame(self):
        """Xử lý khung hình từ hàng đợi và trả về một từ điển chứa thông tin cần thiết."""
        frame_data = self.frame_queue.get()  # Lấy dữ liệu từ hàng đợi
        keys = [
            "frame_color", "mask_red", "mask_yellow", "status_red", "deviation_x_red", 
            "deviation_y_red", "status_yellow", "deviation_x_yellow", "deviation_y_yellow", 
            "contours_yellow", "contours_red", "status_water", "status_charger", 
            "deviation_x_water", "deviation_y_water", "deviation_x_charger", "deviation_y_charger", 
            "frame_object"
        ]
        
        # Tạo từ điển từ dữ liệu
        frame_dict = dict(zip(keys, frame_data))
        return frame_dict

    ################################ Các hàm liên quan đến xử lý màu sắc ################################
    def handle_yellow_line(self, status_yellow, deviation_x_yellow, deviation_y_yellow, left_distance, right_distance):
        if status_yellow:
            self.top_servo.move_to_angle(110)
            self.bottom_servo.move_to_angle(60)
            self.update_state("Tránh đường line vàng")
            if left_distance > right_distance: 
                if abs(deviation_x_yellow) < 150 or abs(deviation_y_yellow) < 150:
                    self.set_motors_direction('rotate_left', 0.15, 0.15, 0)
                if deviation_y_yellow > 150 and deviation_x_yellow == 0:
                    self.set_motors_direction('rotate_left', 0.15, 0.15, 0)
            else:
                if abs(deviation_x_yellow) < 150 or abs(deviation_y_yellow) < 150:
                    self.set_motors_direction('rotate_right', 0.15, 0.15, 0)
                if deviation_y_yellow > 150 and deviation_x_yellow == 0:
                    self.set_motors_direction('rotate_right', 0.15, 0.15, 0)
                    
    def red_line_following(self, contours):
        if contours:
            self.update_state("Đang bám theo line đỏ")
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # Điều khiển robot dựa trên vị trí của đường đỏ
                if cx < 100:  
                    self.set_motors_direction('rotate_left', self.vx , self.vy, 0)              
                elif cx > 300:  
                    self.set_motors_direction('rotate_right', self.vx , self.vy, 0)
                else:  
                    self.set_motors_direction('go_forward', self.vx, self.vy, 0)
    
    def get_sensor_data(self):
        """Lấy dữ liệu cảm biến."""
        return {
            "front": self.ultrasonic_sensors.get_distance("front"),
            "left": self.ultrasonic_sensors.get_distance("left"),
            "right": self.ultrasonic_sensors.get_distance("right"),
            "front_left": self.ultrasonic_sensors.get_distance("front_left"),
            "front_right": self.ultrasonic_sensors.get_distance("front_right")
        }
    ################################ Các hàm liên quan đến điều khiển tự động ################################
    def automatic_mode(self):
        self.n = 2
        self.bottom_servo.move_to_angle(self.DEFAULT_ANGLE_BOTTOM)
        self.top_servo.move_to_angle(self.DEFAULT_ANGLE_TOP)
        self.start_video_stream()  # Khởi động luồng video
        color_thread = self.start_color_detection()  # Bắt đầu luồng nhận diện màu
        object_thread = self.start_object_detection()  # Bắt đầu luồng nhận diện đối tượng

        search_thread = None  # Biến để theo dõi luồng tìm kiếm
        last_detection_time = time()  # Thời gian phát hiện vật cuối cùng
        search_interval = 12  # Thời gian tìm kiếm lại (60 giây)

        print("Chế độ tự động đang chạy...")
        try:
            while not self.stop_event.is_set():  # Vòng lặp chính
                self.daily_reset_check()
                if not self.manual_mode:
                    current_time = time()
                    
                    # Xử lý khung hình từ hàng đợi
                    if not self.frame_queue.empty():
                        frame_dict = self.process_frame()
                        
                        front_distance = self.ultrasonic_sensors.get_distance("front")
                        left_distance = self.ultrasonic_sensors.get_distance("left")
                        right_distance = self.ultrasonic_sensors.get_distance("right")
                        front_left_distance = self.ultrasonic_sensors.get_distance("front_left")
                        front_right_distance = self.ultrasonic_sensors.get_distance("front_right")
                        if self.firtstart:
                            sleep(3)
                            self.firtstart = False 
            
                        # Lấy các giá trị từ từ điển
                        status_red = frame_dict["status_red"]
                        status_yellow = frame_dict["status_yellow"]
                        deviation_x_yellow = frame_dict["deviation_x_yellow"]
                        deviation_y_yellow = frame_dict["deviation_y_yellow"]
                        status_charger = frame_dict["status_charger"]
                        deviation_x_charger = frame_dict["deviation_x_charger"]
                        deviation_y_charger = frame_dict["deviation_y_charger"]
                        deviation_x_water = frame_dict["deviation_x_water"]
                        deviation_y_water = frame_dict["deviation_y_water"]
                        status_water = frame_dict["status_water"]
                        contours_red = frame_dict["contours_red"]
                        frame_object = frame_dict["frame_object"]
                        mask_red = frame_dict["mask_red"]
                        mask_yellow = frame_dict["mask_yellow"]
                        frame_color = frame_dict["frame_color"]
                        
                        # if frame_object is not None:
                        #     cv2.imshow("object detection", frame_object)
                        #     cv2.waitKey(1)
                        
                        print(self.current_state)
                        
                        self.return_to_charger_station(status_yellow, deviation_x_yellow, deviation_y_yellow, left_distance, right_distance,
                            status_charger, status_red, contours_red, front_distance, front_left_distance, front_right_distance)
                        
                        last_detection_time = self.handle_water_mission(
                            status_yellow, deviation_x_yellow, deviation_y_yellow, 
                            left_distance, right_distance, status_water, 
                            deviation_x_water, deviation_y_water, front_distance, 
                            current_time, last_detection_time, search_interval, 
                            search_thread, front_left_distance, front_right_distance
                        )
                    sleep(0.05)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.stop_event.set()
                        break
                else:
                    self.update_state("Chuyển sang chế độ thủ công. Thoát khỏi tự động.")
                    break  # Thoát khỏi vòng lặp nếu chuyển sang chế độ thủ công
            color_thread.join()
            object_thread.join()

        except Exception as e:
            print(f"Error in automatic mode: {e}")

        finally:
            self.stop_event.set()
            if 'videostream' in locals():
                self.videostream.stop()  # Dừng video stream
            cv2.destroyAllWindows()  # Đóng tất cả cửa sổ OpenCV
    #######################################Main############################################
    def handle_water_mission(self, status_yellow, deviation_x_yellow, deviation_y_yellow, left_distance, right_distance,
                         status_water, deviation_x_water, deviation_y_water, front_distance, 
                         current_time, last_detection_time, search_interval, search_thread,
                         front_left_distance, front_right_distance):
        """
        Handles the mission of watering plants, including line following, plant detection, and obstacle avoidance.
        """
        # Xử lý khi phát hiện đường màu vàng
        if status_yellow and not self.search_object and self.current_state != "watering":
            self.handle_yellow_line(status_yellow, deviation_x_yellow, deviation_y_yellow, left_distance, right_distance)
            self.current_state = "avoid_line"

        # Khi không có đường màu vàng
        elif not status_yellow: 
            self.check_tracking_water(status_water)
            if status_water:  # Nếu phát hiện cây cần tưới
                last_detection_time = current_time
                if search_thread and search_thread.is_alive():
                    self.stop_search_thread()
                self.move_to_target(deviation_x_water, deviation_y_water, front_distance, right_distance, left_distance)
                last_detection_time = current_time
                self.current_state = "watering"

            elif current_time - last_detection_time >= search_interval:  # Nếu đến thời gian quét
                last_detection_time = current_time
                if not search_thread or not search_thread.is_alive():
                    print("Không phát hiện cây cần tưới. Bắt đầu quét lại...")
                    self.search_object = True
                    self.start_search_thread(self.MIN_ANGLE, 2, 11)
                self.current_state = "searching"
                
            elif self.search_object and not self.is_tracking_warter:  # Khi đang tìm kiếm
                last_detection_time = current_time
                self.update_state("Dừng lại để quét đối tượng")
                self.set_motors_direction("stop", self.vx, self.vy, 0)
                self.current_state = "idle"
            
            else:  # Mặc định điều hướng tránh chướng ngại vật
                if not self.is_tracking_warter:
                    self.avoid_and_navigate(front_distance, left_distance, right_distance, front_left_distance, front_right_distance)
                    self.current_state = "move"

        return last_detection_time
    
    def return_to_charger_station(self, status_yellow, deviation_x_yellow, deviation_y_yellow, left_distance, right_distance,
                         status_charger, status_red, contours_red, front_distance, front_left_distance, front_right_distance):
        
        if status_yellow:
            self.handle_yellow_line(status_yellow, deviation_x_yellow, deviation_y_yellow, left_distance, right_distance)
            self.current_state = "avoid_line"
        # Khi không có đường màu vàng
        elif not status_yellow:
            if status_charger:
                self.rest_in_charger(front_distance)
            elif status_red:
                self.red_line_following(contours_red)
                self.current_state = "following red line"  
            else:
                self.avoid_and_navigate(front_distance, left_distance, right_distance, front_left_distance, front_right_distance)
                self.current_state = "move"
         
    ################################ Charging station ###############################3
    def rest_in_charger(self, front_distance):
        """Đưa robot vào trạng thái nghỉ ngơi tại trạm sạc."""
        if front_distance > 5:
            self.update_state("Di chuyển về phía trước để gần trạm sạc")
            self.set_motors_direction("go_forward", self.vx, self.vy, 0)
            sleep(0.1)
            self.set_motors_direction("stop", self.vx, self.vy, 0)
        else:
            self.set_motors_direction("stop", self.vx, self.vy, 0)  # Dừng động cơ
            self.update_direction("Dừng di chuyển")
            self.update_state("Đang nghỉ ngơi tại trạm sạc")
            self.is_resting = True
        
    def left_charger(self):
        """Đi lùi và quay sang trái để vào trạm sạc."""
        self.set_motors_direction('go_backward', self.vx, self.vy, 0)  # Đi lùi
        self.update_direction("Đi lùi")
        sleep(1)  # Chờ 1 giây
        left_distance = self.ultrasonic_sensors.get_distance('left')
        right_distance = self.ultrasonic_sensors.get_distance('right')
        # So sánh khoảng cách từ cảm biến siêu âm bên trái và bên phải
        if left_distance > right_distance:
            self.set_motors_direction('rotate_left', self.vx, self.vy, 0)  # Quay sang trái
            self.update_direction("Xoay trái")
        else:
            self.set_motors_direction('rotate_right', self.vx, self.vy, 0)  # Quay sang phải
            self.update_direction("Xoay phải")
        sleep(1)
        self.is_resting = False
    #################################Object Tracking###########################################
    def water_plants(self, right_distance, left_distance):
        self.update_state("Đang thực hiện tưới cây")
        self.relay_control.run_relay_for_duration()
        print("Relay activated for watering...")
        self.current_mission_count += 1  # Tăng số lượng cây đã tưới
        self.check_daily_mission()
        self.watered = True
        if right_distance > left_distance:
            self.rotate_robot('rotate_right')
            self.update_direction("Xoay phải")
        else:
            self.rotate_robot('rotate_left')
            self.update_direction("Xoay trái")
        self.reset_servo_to_default()
        self.watered = False
        
    def rotate_robot_tracking(self, target_angle):
        self.update_state("Điều chỉnh vị trí cho việc tracking")
        if target_angle < self.DEFAULT_ANGLE_BOTTOM - 6:
            self.set_motors_direction('rotate_right', self.vx, self.vy, 1)
            self.update_direction("Xoay phải điều chỉnh góc")
            sleep(0.2)
            self.set_motors_direction('stop', self.vx, self.vy, 1)
        elif target_angle > self.DEFAULT_ANGLE_BOTTOM + 6:
            self.set_motors_direction('rotate_left', self.vx, self.vy, 1)
            self.update_direction("Xoay trái điều chỉnh góc")
            sleep(0.2)
            self.set_motors_direction('stop', self.vx, self.vy, 1)
            
    def move_to_target(self, deviation_x, deviation_y, front_distance, right_distance, left_distance):
        self.update_state("Đang tracking đối tượng")
        self.update_state("tracking")
        print(deviation_x, deviation_y)
        bottom_angle = self.bottom_angle  # Sử dụng giá trị góc hiện tại của servo dưới
        top_angle = self.top_angle  # Sử dụng giá trị góc hiện tại của servo trên

        # Điều chỉnh góc servo dưới
        if deviation_x <= -12:
            bottom_angle += 1
            if bottom_angle <= self.MAX_ANGLE:
                self.bottom_servo.move_to_angle(bottom_angle)
                self.servo_angle_history_bottom.append(bottom_angle)
                if len(self.servo_angle_history_bottom) > self.MAX_HISTORY:
                    self.servo_angle_history_bottom.pop(0)
        elif deviation_x >= 12:
            bottom_angle -= 1
            if bottom_angle >= self.MIN_ANGLE:
                self.bottom_servo.move_to_angle(bottom_angle)
                self.servo_angle_history_bottom.append(bottom_angle)
                if len(self.servo_angle_history_bottom) > self.MAX_HISTORY:
                    self.servo_angle_history_bottom.pop(0)
        # Điều chỉnh góc servo trên
        if deviation_y <= -12:
            top_angle -= 1
            if top_angle <= self.MAX_ANGLE:
                self.top_servo.move_to_angle(top_angle)
                self.servo_angle_history_top.append(top_angle)
                if len(self.servo_angle_history_top) > self.MAX_HISTORY:
                    self.servo_angle_history_top.pop(0)
        elif deviation_y >= 12:
            top_angle += 1
            if top_angle >= self.MIN_ANGLE:
                self.top_servo.move_to_angle(top_angle)
                self.servo_angle_history_top.append(top_angle)
                if len(self.servo_angle_history_top) > self.MAX_HISTORY:
                    self.servo_angle_history_top.pop(0)

        # Lưu lại góc hiện tại sau khi điều chỉnh
        self.bottom_angle = bottom_angle
        self.top_angle = top_angle

        if (15 < abs(deviation_x) <= 50 and 15 < abs(deviation_y) <= 50) or ( 30 < bottom_angle > 90)  :
            self.rotate_robot_tracking(self.bottom_angle)
            
        elif front_distance <= 17:
            self.set_motors_direction('stop', self.vx, self.vy, 0)
            self.update_state("Xe đã tới gần vật, dừng lại.")
            self.water_plants(right_distance, left_distance)
            
        elif front_distance > self.SAFE_DISTANCE and abs(deviation_x) <= 12 and abs(deviation_y) <= 12:
            if len(self.servo_angle_history_bottom) >= 3:
                last_three_angles = self.servo_angle_history_bottom[-3:]
                if (all(54 <= angle <= 66 for angle in last_three_angles)) or (bottom_angle < 75 and abs(deviation_x) <= 10 and abs(deviation_y) <= 10):
                    self.update_state("Servo 1 ổn định, robot bắt đầu di chuyển!")
                    self.set_motors_direction('go_forward', self.vx, self.vy, 0)
                    sleep(0.5)
                    self.set_motors_direction('stop', self.vx, self.vy, 0)
                
    ################################ ?????????????? ###############################
    def reset_servo_to_default(self):
        print("Đưa servo về góc mặc định.")
        self.bottom_servo.move_to_angle(self.DEFAULT_ANGLE_BOTTOM)
        self.top_servo.move_to_angle(self.DEFAULT_ANGLE_TOP)
        
    def check_tracking_charger(self, status_charger):
        # Thêm trạng thái hiện tại vào mảng
        self.status_charger_history.append(status_charger)
        if len(self.status_charger_history) > self.reset_threshold:
            self.status_charger_history.pop(0)  # Xóa trạng thái cũ nhất
        # Kiểm tra nếu không có giá trị True trong 3 giá trị gần nhất
        if all(not status for status in self.status_charger_history):
            self.reset_servo_to_default()  # Reset servo nếu không có giá trị True 
            
    def check_tracking_water(self, status_water):
        self.status_water_history.append(status_water)
        if len(self.status_water_history) > self.reset_threshold:
            self.status_water_history.pop(0)  # Xóa trạng thái cũ nhất
        if all(not status for status in self.status_water_history):
            self.is_tracking_warter = False  # Không theo dõi nữa
        else:
            self.is_tracking_warter = True
            
    ############################Kiểm tra thời gian nhiệm vụ ##############################
    def check_battery_and_time(self):
        """Kiểm tra trạng thái pin và thời gian hoạt động."""
        now = datetime.now()  # Lấy thời gian hiện tại dưới dạng datetime
        if self.OPERATION_END_TIME <= now >= self.OPERATION_START_TIME:
            print(f"Đã đến giờ hoạt động. Bắt đầu vào lúc {self.OPERATION_START_TIME}.")
            return self.battery.read_battery_status()[3] > 25 
        return False
    
    def reset_daily_mission(self):
        """Reset biến daily_mission và current_mission_count mỗi ngày."""
        self.current_mission_count = 0
        self.mission = True  # Đặt lại mission thành True
        print("Đã reset nhiệm vụ hàng ngày.")

    def check_daily_mission(self):
        """Kiểm tra xem đã hoàn thành nhiệm vụ hàng ngày chưa."""
        if self.current_mission_count >= self.daily_mission:
            self.mission = False
            print("Hoàn thành nhiệm vụ tưới cây trong ngày.")
    
    def daily_reset_check(self):
        """Kiểm tra thời gian để reset nhiệm vụ hàng ngày."""
        current_time = datetime.now()
        # Giả sử bạn muốn reset vào lúc 00:00
        if current_time.hour == 0 and current_time.minute == 0:
            self.reset_daily_mission()
            
################################## Quét đối tượng trên map #############################
    def search_for_object(self, start_angle, step_angle, number):
        """Hàm quét tìm đối tượng."""
        self.update_state("Đang quét tìm kiếm đối tượng...")
        bottom_angle = start_angle
        top_angle = 110

        while not self.stop_search_event.is_set():  # Kiểm tra nếu cần dừng tìm kiếm
            while top_angle >= 50:
                self.bottom_servo.move_to_angle(bottom_angle)
                self.top_servo.move_to_angle(top_angle)
                sleep(0.1)  # Giảm thời gian chờ

                if not self.frame_queue.empty():
                    frame_data = self.frame_queue.get()
                    status = frame_data[number]
                    if status or self.is_tracking_warter:
                        self.update_state(f"Đối tượng phát hiện tại góc ({bottom_angle}, {top_angle})")
                        self.top_angle = top_angle
                        self.bottom_angle = bottom_angle
                        self.result_queue.put((bottom_angle, top_angle))  # Đẩy kết quả vào hàng đợi
                        self.search_object = False
                        return  # Kết thúc tìm kiếm khi phát hiện đối tượng

                # Cập nhật góc quét
                if bottom_angle < self.MAX_ANGLE:
                    bottom_angle += step_angle
                else:
                    bottom_angle = self.MIN_ANGLE
                    top_angle -= 30  # Giảm góc của servo trên
             
            self.search_object = False       
            self.is_tracking_warter = False
            self.reset_servo_to_default()
            self.update_state("Không phát hiện được đối tượng.")
            self.result_queue.put(None)  # Đẩy None vào hàng đợi khi không phát hiện được đối tượng
            
    def start_search_thread(self, start_angle, step_angle, number):
        self.stop_search_event.clear()  # Đảm bảo cờ dừng được reset
        self.search_thread = Thread(target=self.search_for_object, args=(start_angle, step_angle, number), daemon=True)
        self.search_thread.start()

    def stop_search_thread(self):
        """Dừng luồng tìm kiếm."""
        self.stop_search_event.set()  # Đặt cờ dừng
        if self.search_thread and self.search_thread.is_alive():
            self.search_thread.join()  # Đợi luồng kết thúc
    ##################################gggggggggggggg######################################