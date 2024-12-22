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
        self.Mission = True
        
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

    def avoid_and_navigate(self, front_distance, left_distance, right_distance,front_left_distance,front_right_distance):

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
    def cv_show(self, frame, name="frame"):
        """Hiển thị khung hình bằng OpenCV."""
        cv2.imshow(name, frame)
        cv2.waitKey(1)
    def process_frame(self):
        """Xử lý khung hình từ hàng đợi và trả về một từ điển chứa thông tin cần thiết."""
        frame_data = self.frame_queue.get()  # Lấy dữ liệu từ hàng đợi
        keys = [
            "frame_color", "mask_red", "mask_black", "status_red", "deviation_x_red", 
            "deviation_y_red", "status_black", "deviation_x_black", "deviation_y_black", 
            "contours_black", "contours_red", "status_water", "status_charger", 
            "deviation_x_water", "deviation_y_water", "deviation_x_charger", "deviation_y_charger", 
            "frame_object"
        ]
        
        # Tạo từ điển từ dữ liệu
        frame_dict = dict(zip(keys, frame_data))
        return frame_dict
        
    ################################ Các hàm liên quan đến xử lý màu sắc ################################
    def handle_black_line(self, status_black, deviation_x_black, deviation_y_black):
        if status_black:
            if abs(deviation_x_black) < 200 or abs(deviation_y_black) < 200:
                set_motors_direction('rotate_left', 0.15, 0.15, 0)
            if deviation_y_black > 30 and deviation_x_black == 0:
                set_motors_direction('rotate_right', 0.15, 0.15, 0)
    
    def red_line_following(self, contours):
        print("Theo dõi đường đỏ")
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # Điều khiển robot dựa trên vị trí của đường đỏ
                if cx < 100:  
                    set_motors_direction('rotate_left', 0.1, 0, 0)              
                elif cx > 450:  
                    set_motors_direction('rotate_right', 0.1, 0, 0)
                else:  
                    set_motors_direction('go_forward', 0.1, 0, 0)
        else:
            set_motors_direction('stop', self.vx, self.vy, 0)
        
    ################################ Các hàm liên quan đến điều khiển tự động ################################
    def automatic_mode(self):
        self.start_video_stream()  # Khởi động luồng video
        color_thread = self.start_color_detection()  # Bắt đầu luồng nhận diện màu
        object_thread = self.start_object_detection()  # Bắt đầu luồng nhận diện đối tượng
        last_scan_time = time.time()  # Thời gian quét cuối cùng
        scan_interval = 60  # Thời gian quét định kỳ (60 giây)
    
        try:
            while not self.stop_event.is_set():  # Vòng lặp chạy khi không dừng và không ở chế độ thủ công
                if not self.manual_mode:
                    """Chế độ tự động cho robot thực hiện nhiệm vụ."""
                    print("Chế độ tự động đang chạy...")
                    if not self.frame_queue.empty():
                        frame_dict = self.process_frame()  # Xử lý khung hình từ hàng đợi
                        
                        front_distance = self.ultrasonic_sensors.get_distance("front")
                        left_distance = self.ultrasonic_sensors.get_distance("left")
                        right_distance = self.ultrasonic_sensors.get_distance("right")
                        front_left_distance = self.ultrasonic_sensors.get_distance("front_left")
                        front_right_distance = self.ultrasonic_sensors.get_distance("front_right")
                        
                        # Lấy các giá trị từ từ điển
                        status_black = frame_dict["status_black"]
                        deviation_x_black = frame_dict["deviation_x_black"]
                        deviation_y_black = frame_dict["deviation_y_black"]
                        status_charger = frame_dict["status_charger"]
                        deviation_x_charger = frame_dict["deviation_x_charger"]
                        deviation_y_charger = frame_dict["deviation_y_charger"]
                        deviation_x_water = frame_dict["deviation_x_water"]
                        deviation_y_water = frame_dict["deviation_y_water"]
                        status_water = frame_dict["status_water"]
                        contours_red = frame_dict["contours_red"]
                        
                        if self.check_battery_and_time(time()):
                            if not self.Mission:
                                if front_distance < 10 and status_charger:
                                    self.rest_in_charger()  # Trở về trạm sạc
                                else:
                                    if status_black:
                                        self.handle_black_line(status_black, deviation_x_black, deviation_y_black)
                                    if not status_black:
                                        self.avoid_and_navigate(front_distance, left_distance, right_distance, front_left_distance, front_right_distance)
                                        self.red_line_following(contours_red)
                                        if status_charger:
                                            self.move_to_target(deviation_x_charger, deviation_y_charger, self.DEFAULT_ANGLE_BOTTOM, self.DEFAULT_ANGLE_TOP, front_distance)
                            else:
                                current_time = time()
                                if current_time - last_scan_time >= scan_interval:
                                    # Gọi phương thức quét camera xung quanh
                                    detected_angles = self.search_for_object(self.DEFAULT_ANGLE_BOTTOM, 60, 30, 4, 11)
                                    last_scan_time = current_time  # Cập nhật thời gian quét cuối cùng
                                    if detected_angles[0] is not None:
                                        self.bottom_servo.move_to_angle(detected_angles[0])  # Đưa servo 1 về góc mặc định
                                        self.top_servo.move_to_angle(detected_angles[1])  # Đưa servo 2 về góc mặc định
                                if status_black:
                                    self.handle_black_line(status_black, deviation_x_black, deviation_y_black)
                                if not status_black:
                                    self.avoid_and_navigate(front_distance, left_distance, right_distance, front_left_distance, front_right_distance)
                                    if status_water:
                                        self.move_to_target(deviation_x_water, deviation_y_water, self.DEFAULT_ANGLE_BOTTOM, self.DEFAULT_ANGLE_TOP, front_distance)     
                        else:
                            self.update_state("Battery low, returning to charge station...")
                            if front_distance < 10 and status_charger:
                                self.rest_in_charger()  # Trở về trạm sạc
                            else:
                                self.handle_black_line(status_black, deviation_x_black, deviation_y_black)
                                if not status_black:
                                    self.avoid_and_navigate(front_distance, left_distance, right_distance, front_left_distance, front_right_distance)
                                    self.red_line_following(contours_red)
                                    if status_water:
                                        self.move_to_target(deviation_x_charger, deviation_y_charger, self.DEFAULT_ANGLE_BOTTOM, self.DEFAULT_ANGLE_TOP, front_distance)
                    sleep(0.05)  # Giúp vòng lặp không chiếm quá nhiều tài nguyên
                else:
                    print("Chuyển sang chế độ thủ công. Thoát khỏi tự động.")
                    self.update_state("Automatic exited")
                    break  # Thoát khỏi vòng lặp nếu chuyển sang chế độ thủ công

                # Thêm log để theo dõi trạng thái
                print(f"Manual Mode: {self.manual_mode}, Stop Event: {self.stop_event.is_set()}")
                
            color_thread.join()
            object_thread.join()

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
    
    def rest_in_charger(self):
        """Đưa robot vào trạng thái nghỉ ngơi tại trạm sạc."""
        print("Đang nghỉ ngơi tại trạm sạc")
        self.bottom_servo.move_to_angle(self.DEFAULT_ANGLE_BOTTOM)  # Đưa servo 1 về góc mặc định
        self.top_servo.move_to_angle(self.DEFAULT_ANGLE_TOP)  # Đưa servo 2 về góc mặc định
        self.motors.stop_all()  # Dừng động cơ
        
    def left_charger(self):
        """Đi lùi và quay sang trái để vào trạm sạc."""
        set_motors_direction('go_backward', self.vx, self.vy, 0)  # Đi lùi
        sleep(1)  # Chờ 1 giây
        left_distance = self.ultrasonic_sensors.get_distance('left')
        right_distance = self.ultrasonic_sensors.get_distance('right')
        # So sánh khoảng cách từ cảm biến siêu âm bên trái và bên phải
        if left_distance > right_distance:
            set_motors_direction('rotate_left', self.vx, self.vy, 0)  # Quay sang trái
        else:
            set_motors_direction('rotate_right', self.vx, self.vy, 0)  # Quay sang phải
        sleep(1)
    def return_to_charge_station(self):
        """Đưa robot trở về trạm sạc."""
        print("Đang trở về trạm sạc")
        
        # Giả sử robot có cảm biến để xác định vị trí trạm sạc
        while not self.is_at_charge_station():
            front_distance = self.ultrasonic_sensors.get_distance('front')
            
            if front_distance > 10:  # Nếu khoảng cách phía trước lớn hơn 10 cm
                set_motors_direction('go_forward', self.vx, self.vy, 0)  # Đi thẳng
            else:
                set_motors_direction('stop', self.vx, self.vy, 0)  # Dừng lại
                set_motors_direction('rotate_left', self.vx, self.vy, 0)  # Quay sang trái
                sleep(1)  # Chờ 1 giây
        
        self.rest_in_charger()  # Đưa robot vào trạng thái nghỉ ngơi tại trạm sạc

    def is_at_charge_station(self):
        """Kiểm tra xem robot đã ở trạm sạc chưa."""
        # Giả sử có một cảm biến để kiểm tra xem robot đã ở trạm sạc chưa
        return self.ultrasonic_sensors.get_distance('front') < 5  # Ví dụ: khoảng cách dưới 5 cm
    def tracking_servo_bottom(self, deviation_x, target_angle):
        if deviation_x < -10 and target_angle < self.MAX_ANGLE:
            target_angle += 1
            self.bottom_servo.move_to_angle(target_angle)
        elif deviation_x > 10 and target_angle > self.MIN_ANGLE:
            target_angle -= 1
            self.bottom_servo.move_to_angle(target_angle)

    def tracking_servo_top(self, deviation_y, target_angle):
        if deviation_y < -10 and target_angle > self.MIN_ANGLE:
            target_angle -= 1
            self.top_servo.move_to_angle(target_angle)
        elif deviation_y > 10 and target_angle < self.MAX_ANGLE:
            target_angle += 1
            self.top_servo.move_to_angle(target_angle)
            
    def move_to_target(self, deviation_x, deviation_y, target_angle_bottom, target_angle_top, front_distance):
        # Điều chỉnh servo để theo dõi mục tiêu
        self.tracking_servo_bottom(deviation_x, target_angle_bottom)
        self.tracking_servo_top(deviation_y, target_angle_top)

        # Kiểm tra khoảng cách phía trước để xác định hành động
        if front_distance > 15:
            if 51 < target_angle_bottom < 69 and abs(deviation_x) <= 10:
                set_motors_direction('go_forward', self.vx, self.vy, 0)
                self.update_state("moving forward")
        else:
            set_motors_direction('stop', self.vx, self.vy, 0)
            self.update_state("stop and watering")
            self.relay_control.run_relay_for_duration()

        if abs(deviation_x) > 15:
            self.rotate_robot(target_angle_bottom, self.DEFAULT_ANGLE_BOTTOM, 4)
            self.update_state("rotating to target")

    def rotate_robot(self, target_angle, default_angle, angle):
        if target_angle < default_angle - angle:
            set_motors_direction('rotate_right', 0.1, 0.1, 0)
        elif target_angle > default_angle + angle:
            set_motors_direction('rotate_left', 0.1, 0.1, 0)

    def search_for_object(self, start_angle_bottom, start_angle_top, step_angle, num_turns, n):
        """Tìm kiếm đối tượng bằng cách quay servo."""
        target_angle_bottom = start_angle_bottom
        target_angle_top = start_angle_top

        MAX_ANGLE = 120  # Giới hạn góc tối đa
        MIN_ANGLE = 0    # Giới hạn góc tối thiểu

        for turn in range(num_turns):
            print(f"Vòng tìm kiếm {turn + 1}/{num_turns} ở góc {target_angle_bottom} độ.")

            # Quay servo đến góc hiện tại
            self.bottom_servo.move_to_angle(target_angle_bottom)
            self.top_servo.move_to_angle(target_angle_top)
            sleep(1)  # Chờ một chút để servo ổn định

            # Kiểm tra có phát hiện đối tượng không từ frame_queue
            if not self.frame_queue.empty():
                frame_data = self.frame_queue.get()  # Lấy thông tin từ hàng đợi
                status = frame_data[n]  # Giả sử status là phần tử thứ 4 trong tuple

                if status:
                    print("Đối tượng đã được phát hiện.")
                    return target_angle_bottom, target_angle_top  # Trả về góc của servo

            # Cập nhật góc quay
            target_angle_bottom += step_angle

            # Giới hạn góc quay
            if target_angle_bottom > MAX_ANGLE:
                target_angle_bottom = MIN_ANGLE  # Reset về góc tối thiểu nếu vượt quá tối đa

        print("Không phát hiện được đối tượng trong vòng tìm kiếm.")
        return None, None  # Không tìm thấy đối tượng
