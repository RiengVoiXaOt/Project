from time import sleep, time
from src.hardware.relay import RelayControl
from src.hardware.motors import Motors
from src.hardware.ultrasonic import UltrasonicSensors
from src.vision.color_detection import color_detection_loop
from src.vision.object_detection import object_detection_loop
from src.utils.control_utils import getch, direction, set_motors_direction
import numpy as np
from threading import Thread, Event
import os
import cv2
import numpy as np
from threading import Thread, Event
from queue import Queue
from src.vision.video_stream import VideoStream
from src.hardware.servos import ServoControl, MIN_ANGLE, MAX_ANGLE, DEFAULT_ANGLE
from src.utils.image_utils import load_labels, load_model, get_model_details, draw_detections, preprocess_frame, calculate_fps, analyze_detection
import warnings
from src.vision.object_detection import object_detection_loop  # Nhập hàm từ module bên ngoài

warnings.filterwarnings("ignore", category=UserWarning, module='cv2')

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
    # Định nghĩa các hằng số và thông số toàn cục
    SAFE_DISTANCE = 15  # Khoảng cách an toàn (cm)
    CRITICAL_DISTANCE = 12  # Ngưỡng cảnh báo (cm)
    MAX_HISTORY = 10  # Giới hạn lịch sử khoảng cách
    MODEL_NAME = '/home/az/Desktop/Project/az_project_robot/models'
    GRAPH_NAME = 'detect.tflite'
    LABELMAP_NAME = 'labelmap.txt'
    MIN_CONF_THRESHOLD = 0.7
    IM_WIDTH, IM_HEIGHT = 640, 480
    CENTER_X, CENTER_Y = IM_WIDTH // 2, IM_HEIGHT // 2
    PATH_TO_CKPT = os.path.join(MODEL_NAME, GRAPH_NAME)
    PATH_TO_LABELS = os.path.join(MODEL_NAME, LABELMAP_NAME)

    def __init__(self, n=None, theta=None):
        # Khởi tạo các thành phần phần cứng của robot
        self.relay_control = RelayControl(5)  # Điều khiển relay cho tưới cây
        self.top_servo = ServoControl(channel=0)  # Sử dụng servo_2 cho servo trên
        self.bottom_servo = ServoControl(channel=1)  # Sử dụng servo_1 cho servo dưới
        self.motors = Motors()  # Điều khiển động cơ
        self.ultrasonic_sensors = UltrasonicSensors()  # Cảm biến siêu âm
        self.manual_mode = False  # Biến kiểm tra chế độ thủ công
        self.is_watering = False  # Biến kiểm tra trạng thái tưới cây
        self.charge_station_found = False  # Biến kiểm tra trạng thái tìm thấy trạm sạc
        if n is None:
            n = 5
        self.n = n  # Khởi tạo tốc độ ở mức tối thiểu
        if theta is None:
            theta = 0  # Góc quay mặc định
        self.speed = 0.04309596457  # Tốc độ tối thiểu (10% tốc độ tối đa)
        self.vx = self.n * self.speed  # Tốc độ theo hướng x
        self.vy = self.n * self.speed  # Tốc độ theo hướng y
        self.theta = theta  # Góc quay
        self.state = "stopped"  # Trạng thái hiện tại của robot
        self.distance_history = {'front': [], 'left': [], 'right': []}  # Lịch sử khoảng cách
        self.active_mode = "automatic"  # Biến theo dõi chế độ hoạt động hiện tại
        self.last_activity_time = time()  # Thời gian hoạt động cuối cùng
        self.check_event = Event()  # Để kiểm soát việc kiểm tra chế độ
        self.frame_queue = Queue(maxsize=1)  # Hàng đợi khung hình
        self.videostream = VideoStream(resolution=(self.IM_WIDTH, self.IM_HEIGHT), framerate=30)
        self.stop_event = Event()  # Biến để dừng luồng
        self.is_running = False  # Trạng thái chạy
        
        # Khởi động luồng video
        self.videostream.start()
        time.sleep(1)
        ###############################Không cần chỉnh sửa các hàm này####################
    def switch_mode(self):
        """Chuyển đổi giữa chế độ tự động và chế độ thủ công."""
        self.manual_mode = not self.manual_mode  # Đảo ngược chế độ
        mode = "manual" if self.manual_mode else "automatic"
        print(f"Switched to {mode} mode.")
        if self.manual_mode:
            self.motors.stop_all()  # Dừng tất cả động cơ khi chuyển sang chế độ thủ công
            
    def start(self):
        """Bắt đầu chế độ tự động."""
        self.check_event.clear()
        automatic_thread = Thread(target=self.automatic_mode)
        automatic_thread.start()
        
    def manual_control(self):
        """Chế độ điều khiển thủ công bằng bàn phím."""
        while self.manual_mode:
            print("Enter command (w/a/s/d/q/e/z/x/1/2/3 to move, r to toggle relay, p to quit): ")
            command = getch()  # Nhận lệnh từ bàn phím
            if command == 'p':
                self.top_servo.reset()  # Đặt lại servo
                self.bottom_servo.reset()  # Đặt lại servo
                self.update_state("manual control exited")
                set_motors_direction('stop', 0, 0, 0)
                print("Exiting manual control.")
                return
            elif command in ['+', '=']:
                self.n = min(self.n + 1, 10)  # Tăng tốc độ lên tối đa 100%
                self.vx = self.n * self.speed
                self.vy = self.n * self.speed
                self.update_state(f"speed increased to {self.n * 10}%")
                print(f"Tốc độ tăng lên {self.n * 10}%")
            elif command in ['-', '_']:
                self.n = max(self.n - 1, 0)  # Giảm tốc độ xuống tối thiểu 0%
                self.vx = self.n * self.speed
                self.vy = self.n * self.speed
                self.update_state(f"speed decreased to {self.n * 10}%")
                print(f"Tốc độ giảm xuống {self.n * 10}%")
            elif command in direction:
                current_direction = direction[command]  # Lấy hướng di chuyển từ lệnh
                print("Direction: " + current_direction)
                set_motors_direction(current_direction, self.vx, self.vy, self.theta)  # Thiết lập hướng động cơ
                self.update_state(f"moving {current_direction}")
            elif command == 'r':
                self.relay_control.run_relay_for_duration()  # Bật relay tưới cây
                self.update_state("watering activated")
            elif command in ['7', '8', '9', '0']:
                if command == '7':
                    self.bottom_servo.move_up()  # Di chuyển servo dưới lên
                    self.update_state("bottom servo moved up")
                elif command == '8':
                    self.bottom_servo.move_down()  # Di chuyển servo dưới xuống
                    self.update_state("bottom servo moved down")
                elif command == '9':
                    self.top_servo.move_up()  # Di chuyển servo trên lên
                    self.update_state("top servo moved up")
                elif command == '0':
                    self.top_servo.move_down()  # Di chuyển servo trên xuống
                    self.update_state("top servo moved down")
            else:
                self.update_state("invalid command")
                print("Lệnh không hợp lệ. Vui lòng thử lại.")

    def update_state(self, new_state):
            """Cập nhật trạng thái và in trạng thái mới nếu có thay đổi."""
            if self.state != new_state:
                self.state = new_state
                print(f"Trạng thái hiện tại: {self.state}")

    def move_forward(self):
        set_motors_direction("go_forward", self.vx, self.vy, 0)  # Thiết lập hướng di chuyển
        self.update_state("moving forward")

    def move_backward(self):
        set_motors_direction("go_backward", self.vx, self.vy, 0)  # Thiết lập hướng lùi
        self.update_state("moving backward")

    def stop_robot(self):
        set_motors_direction("stop", 0, 0, 0)  # Dừng tất cả động cơ
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

        if None in (front_distance, left_distance, right_distance):
            print("Lỗi cảm biến siêu âm: Không nhận được dữ liệu.")
            self.stop_robot()
            return

        self.update_distance_history(front_distance, left_distance, right_distance)

        f_state = front_distance < self.SAFE_DISTANCE
        l_state = left_distance < self.SAFE_DISTANCE
        r_state = right_distance < self.SAFE_DISTANCE

        if not f_state:
            if not r_state and not l_state:
                self.move_forward()
            if r_state and right_distance < 15:
                set_motors_direction('go_left', self.vx, self.vy, 0)
                self.update_state("going left")
            elif l_state and left_distance < 15:
                set_motors_direction('go_right', self.vx, self.vy, 0)
                self.update_state("going right")
        else:
            if front_distance <= self.CRITICAL_DISTANCE:
                self.move_backward()
                return

            self.handle_side_obstacles(l_state, r_state, left_distance, right_distance)

    def update_distance_history(self, front, left, right):
        """Lưu lịch sử khoảng cách và giới hạn kích thước."""
        for key, value in zip(['front', 'left', 'right'], [front, left, right]):
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
    
    def start_object_detection(self):
        """Bắt đầu luồng phát hiện đối tượng."""
        self.stop_event.clear()
        object_thread = Thread(target=object_detection_loop, args=(self.videostream, self.stop_event, self.frame_queue, "Water"), daemon=True)
        object_thread.start()

    def stop_object_detection(self):
        """Dừng luồng phát hiện đối tượng."""
        self.stop_event.set()
        self.videostream.stop()
        cv2.destroyAllWindows()

    def search_for_object(self, num_turns=4, step_angle=30, start_angle_1=0, start_angle_2=90):
        """
        Tìm kiếm đối tượng bằng cách quay servo xung quanh từ góc khởi đầu đến góc tối đa.
        """
        print("Bắt đầu tìm kiếm vật thể...")
        
        target_angle_1 = start_angle_1
        target_angle_2 = start_angle_2

        MAX_ANGLE = 120  # Giới hạn góc tối đa
        MIN_ANGLE = 0    # Giới hạn góc tối thiểu
        for turn in range(num_turns):
            print(f"Vòng tìm kiếm {turn + 1}/{num_turns} ở góc {target_angle_1} độ.")

            # Quay servo đến góc hiện tại
            self.bottom_servo.move_to_angle(target_angle_1)
            self.top_servo.move_to_angle(target_angle_2)
            sleep(1)  # Chờ một chút để servo ổn định

            # Kiểm tra có phát hiện đối tượng không từ frame_queue
            if not self.frame_queue.empty():
                status, _, _, _ = self.frame_queue.get()  # Lấy thông tin từ hàng đợi
                if status:
                    print("Đối tượng đã được phát hiện.")
                    return target_angle_1, target_angle_2  # Trả về góc của servo

            # Cập nhật góc quay
            target_angle_1 += step_angle

            # Giới hạn góc quay
            if target_angle_1 > MAX_ANGLE:
                target_angle_1 = MIN_ANGLE  # Reset về góc tối thiểu nếu vượt quá tối đa

        print("Không phát hiện được đối tượng trong vòng tìm kiếm.")
        return None, None  # Không tìm thấy đối tượng

    ###############################Không cần chỉnh sửa các hàm này####################
    
    
    def automatic_mode(self):
        """Chế độ tự động cho robot thực hiện nhiệm vụ."""
        print("Chế độ tự động đang chạy...")
        while not self.manual_mode:  # Vòng lặp chạy khi ở chế độ tự động
            self.avoid_obstacles()  # Tránh chướng ngại vật
            # Gọi các hàm khác để thực hiện nhiệm vụ như tưới cây, tìm kiếm vật thể, v.v.
            
            # Kiểm tra pin và quay về trạm sạc nếu cần
            # if self.should_return_to_charge_station():
            #     print("Pin yếu, đang quay về trạm sạc...")
            #     self.return_to_charge_station()
            #     continue
            
            # Thêm một khoảng thời gian nghỉ để giảm tải CPU
            sleep(0.05)  # Giúp vòng lặp không chiếm quá nhiều tài nguyên

    def exit_automatic_mode(self):
        """Thoát chế độ tự động."""
        print("Đang thoát chế độ tự động...")
        self.manual_mode = True
        self.stop_robot()  # Dừng robot khi thoát chế độ tự động
    
    def activate_automatic_mode(self):
        """Kích hoạt lại chế độ tự động."""
        print("Kích hoạt lại chế độ tự động.")
        self.manual_mode = False
        self.start()  # Bắt đầu lại chế độ tự động
        
    def user_input_listener(self):
        """Lắng nghe đầu vào từ người dùng để thoát chế độ tự động."""
        while True:
            user_input = input("Nhấn 'p' để thoát khỏi chế độ tự động: ")
            if user_input.lower() == 'p':
                self.exit_automatic_mode()
                break
            
    def avoid_black_line(self):
        """Tránh đường line màu đen dựa vào cảm biến siêu âm."""
        front_distance = self.ultrasonic_sensors.get_distance("front")
        left_distance = self.ultrasonic_sensors.get_distance("left")
        right_distance = self.ultrasonic_sensors.get_distance("right")

        if None in (front_distance, left_distance, right_distance):
            print("Lỗi cảm biến siêu âm: Không nhận được dữ liệu.")
            self.stop_robot()
            return

        # Kiểm tra khoảng cách và xác định hướng di chuyển
        distances = {'front': front_distance, 'left': left_distance, 'right': right_distance}
        
        # Tìm hướng có khoảng cách lớn nhất
        max_distance_direction = max(distances, key=distances.get)
        max_distance = distances[max_distance_direction]

        # Kiểm tra khoảng cách tối thiểu an toàn
        if max_distance < self.SAFE_DISTANCE:
            # Nếu khoảng cách không an toàn, thực hiện hành động tương ứng
            if max_distance_direction == 'front':
                self.move_backward()  # Lùi lại nếu khoảng cách phía trước không an toàn
            elif max_distance_direction == 'left':
                set_motors_direction('go_right', self.vx, self.vy, 0)  # Xoay sang phải
                self.update_state("turning right to avoid black line")
            elif max_distance_direction == 'right':
                set_motors_direction('go_left', self.vx, self.vy, 0)  # Xoay sang trái
                self.update_state("turning left to avoid black line")
        else:
            # Nếu khoảng cách phía trước an toàn, tiến về phía trước
            if max_distance_direction == 'front':
                self.move_forward()  # Tiến về phía trước nếu khoảng cách phía trước lớn nhất
            elif max_distance_direction == 'left':
                set_motors_direction('go_right', self.vx, self.vy, 0)  # Xoay sang phải nếu khoảng cách bên trái lớn nhất
                self.update_state("turning right to avoid black line")
            elif max_distance_direction == 'right':
                set_motors_direction('go_left', self.vx, self.vy, 0)  # Xoay sang trái nếu khoảng cách bên phải lớn nhất
                self.update_state("turning left to avoid black line")
            else:
                # Nếu cả ba khoảng cách bằng nhau, ưu tiên xoay phải
                set_motors_direction('go_right', self.vx, self.vy, 0)
                self.update_state("turning right due to equal distances")
        
    def find_object(self):
        """
        Tìm kiếm vật thể bằng cách quét qua servo, di chuyển đến vật thể nếu phát hiện, 
        và bật relay nếu khoảng cách đủ gần.
        """
        print("Bắt đầu tìm kiếm vật thể...")

        # Quét bằng servo trên và dưới
        for top_angle in range(-90, 90, 20):  # Servo trên quét từ 0° đến 180°
            self.top_servo.move_to_angle(top_angle)
            for bottom_angle in range(-90, 90, 20):  # Servo dưới quét từ 0° đến 180°
                self.bottom_servo.move_to_angle(bottom_angle)

                # Phát hiện đối tượng
                detected_objects = object_detection_loop()
                if not detected_objects:
                    continue

                for obj in detected_objects:
                    label = obj.get("label")
                    distance = self.ultrasonic_sensors.get_distance("front")

                    if label == "water":
                        print(f"Phát hiện vật thể '{label}' ở góc ({top_angle}, {bottom_angle})")
                        
                        # Xác định tâm của vật
                        while True:
                            # Cập nhật lại vị trí của vật thể
                            detected_objects = object_detection_loop()
                            if detected_objects:
                                obj = detected_objects[0]  # Lấy đối tượng đầu tiên
                                label = obj.get("label")
                                if label == "water":
                                    # Di chuyển servo trên để theo dõi vật
                                    self.top_servo.move_to_angle(top_angle)
                                    # Cập nhật khoảng cách
                                    distance = self.ultrasonic_sensors.get_distance("front")

                                    # Xoay robot để servo dưới ở vị trí mặc định
                                    self.rotate_robot('rotate_left')  # Hoặc 'rotate_right' tùy vào vị trí
                                    sleep(0.1)  # Một khoảng thời gian ngắn để robot xoay

                                    # Kiểm tra xem robot đã thẳng hàng với vật chưa
                                    if self.is_aligned_with_object(obj):
                                        break  # Thoát vòng lặp nếu đã thẳng hàng

                        # Di chuyển thẳng đến vật thể
                        while distance > 10:  # Di chuyển đến khi khoảng cách <= 10 cm
                            print(f"Khoảng cách đến vật thể: {distance} cm")
                            self.move_forward()
                            distance = self.ultrasonic_sensors.get_distance("front")

                        # Dừng lại và bật relay
                        self.stop_robot()
                        print("Đã đến gần vật thể. Bật relay...")
                        self.relay_control.toggle_relay(True)
                        sleep(2)  # Bật relay trong 2 giây
                        self.relay_control.toggle_relay(False)
                        return  # Thoát hàm sau khi hoàn tất

        print("Không tìm thấy vật thể.")

    def return_to_charger(self):
        """Quay về trạm sạc."""
        if not self.charge_station_found:
            print("Searching for charge station...")
            line_found = color_detection_loop("red")  # Tìm đường line màu đỏ
            if line_found:
                # Logic quay về trạm sạc
                print("Đã phát hiện trạm sạc, quay về...")
                # Giả sử robot quay về theo đường line
                while not self.charge_station_found:
                    # Logic di chuyển về trạm sạc
                    self.move_forward(self.SAFE_DISTANCE)  # Di chuyển về phía trước
                    # Kiểm tra xem có đến trạm sạc không
                    if self.detect_charge_station():  # Hàm này cần được định nghĩa
                        self.charge_station_found = True
                        print("Đã đến trạm sạc.")
                        break
    def detect_charge_station(self):
        """Kiểm tra xem robot có đến trạm sạc hay không."""
        # Logic để phát hiện trạm sạc, ví dụ sử dụng cảm biến hoặc camera
        # Trả về True nếu phát hiện, False nếu không
        return False  # Placeholder, cần thay thế bằng logic thực tế
    
    
    def adjust_direction(self):
        """Điều chỉnh hướng của robot khi phát hiện đường line màu đen."""
        print("Điều chỉnh hướng để tránh đường line màu đen.")
        # Logic để quay robot sang trái hoặc phải
        self.rotate_robot('rotate_right')  # Ví dụ: quay phải

    def water_plants(self):
            """Thực hiện tưới cây."""
            if self.is_watering:
                print("Robot đang tưới cây, không thể tưới thêm.")
                return
            print("Bắt đầu tưới cây...")
            self.is_watering = True  # Đặt trạng thái tưới cây
            self.relay_control.toggle_relay(True)  # Bật relay để tưới
            sleep(10)  # Tưới trong 10 giây
            self.relay_control.toggle_relay(False)  # Tắt relay
            self.is_watering = False  # Đặt lại trạng thái tưới cây
            print("Hoàn thành việc tưới cây.")
            
    def return_to_charge_station(self):
        """Quay về trạm sạc bằng cách nhận diện đường line màu đỏ."""
        print("Đang tìm đường về trạm sạc...")
        while not self.manual_mode:
            if self.sensors.detect_line_red():
                print("Phát hiện đường line màu đỏ, đang đi theo đường...")
                self.follow_line()  # Giả sử có hàm để đi theo đường line
            else:
                print("Không tìm thấy đường line màu đỏ, đang tìm kiếm...")
                self.avoid_obstacles()  # Tránh chướng ngại vật trong khi tìm kiếm
            sleep(0.05)
            
    def follow_line(self):
        """Đi theo đường line màu đỏ."""
        while not self.manual_mode:
            if self.sensors.detect_line_red():
                self.move_forward()
            else:
                self.adjust_direction()  # Điều chỉnh hướng nếu không phát hiện đường line
            
            
    def detect_charge_station(self):
        """Kiểm tra xem robot có đến trạm sạc hay không."""
        # Logic để phát hiện trạm sạc
        distance_to_station = self.ultrasonic_sensors.get_distance("front")
        if distance_to_station is not None and distance_to_station < self.SAFE_DISTANCE:
            print("Phát hiện trạm sạc!")
            return True
        return False
    
    def water_plants(self):
            """Thực hiện tưới cây."""
            if self.is_watering:
                print("Robot đang tưới cây, không thể tưới thêm.")
                return
            print("Bắt đầu tưới cây...")
            self.is_watering = True  # Đặt trạng thái tưới cây
            self.relay_control.toggle_relay(True)  # Bật relay để tưới
            sleep(10)  # Tưới trong 10 giây
            self.relay_control.toggle_relay(False)  # Tắt relay
            self.is_watering = False  # Đặt lại trạng thái tưới cây
            print("Hoàn thành việc tưới cây.")
    def handle_detected_object(self, obj):
        """Xử lý vật thể đã phát hiện."""
        print(f"Phát hiện vật thể: {obj.get('label')}")
        distance = self.ultrasonic_sensors.get_distance("front")
        
        if distance > 10:
            print(f"Khoảng cách đến vật thể: {distance} cm. Đang tiến tới...")
            while distance > 10:
                self.move_forward()
                distance = self.ultrasonic_sensors.get_distance("front")
        
        self.stop_robot()
        print("Đã đến gần vật thể. Bật relay...")
        self.relay_control.toggle_relay(True)
        sleep(2)
        self.relay_control.toggle_relay(False)
        print("Relay đã tắt.")
        
    def get_battery_level(self):
        """Giả sử có một hàm để lấy mức pin."""
        # Logic thực tế để lấy mức pin từ cảm biến hoặc mạch điện
        battery_level = self.read_battery_sensor()  # Giả sử có hàm này
        if battery_level is None:
            print("Không thể đọc mức pin. Sử dụng giá trị mặc định 100%.")
            return 100  # Trả về giá trị mặc định nếu không đọc được
        return battery_level
    
    def should_return_to_charge_station(self):
        """Kiểm tra mức pin để quyết định có quay về trạm sạc hay không."""
        battery_level = self.get_battery_level()
        if battery_level < 20:
            print(f"Mức pin hiện tại: {battery_level}%. Quay về trạm sạc.")
            return True
        elif battery_level < 50:
            print(f"Mức pin hiện tại: {battery_level}%. Cần chú ý, hãy xem xét quay về trạm sạc sớm.")
        return False
    
    def has_plants_to_water(self):
        """Kiểm tra xem có cây nào cần tưới hay không."""
        # Giả sử có một danh sách các cây cần tưới
        return any(plant.needs_watering for plant in self.plants)  # Cần định nghĩa plants