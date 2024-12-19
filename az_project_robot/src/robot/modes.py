from time import sleep, time
from src.hardware.relay import RelayControl
from src.hardware.servos import ServoControl
from src.config.gpio_config import kit
from src.hardware.motors import Motors
from src.hardware.ultrasonic import UltrasonicSensors
from src.vision.color_detection import color_detection_loop
from src.vision.object_detection import object_detection_loop
from src.utils.control_utils import getch, direction, set_motors_direction
import numpy as np

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
    SAFE_DISTANCE = 15  # Khoảng cách an toàn (cm)
    CRITICAL_DISTANCE = 12  # Ngưỡng cảnh báo (cm)
    MAX_HISTORY = 10  # Giới hạn lịch sử khoảng cách

    def __init__(self, n=None, theta=None):
        # Khởi tạo các thành phần phần cứng của robot
        self.relay_control = RelayControl(5)  # Điều khiển relay cho tưới cây
        self.top_servo = ServoControl(kit.servo[7])  # Sử dụng servo_2 cho servo trên
        self.bottom_servo = ServoControl(kit.servo[8])  # Sử dụng servo_1 cho servo dưới
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
    
    def switch_mode(self):
        """Chuyển đổi giữa chế độ tự động và chế độ thủ công."""
        self.manual_mode = not self.manual_mode  # Đảo ngược chế độ
        mode = "manual" if self.manual_mode else "automatic"
        print(f"Switched to {mode} mode.")
        if self.manual_mode:
            self.motors.stop_all()  # Dừng tất cả động cơ khi chuyển sang chế độ thủ công
    
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
                
    def automatic_mode(self):
        """Chế độ tự động cho robot thực hiện nhiệm vụ."""
        print("Chế độ tự động đang chạy...")

        while not self.manual_mode:  # Vòng lặp chạy khi ở chế độ tự động
            # Kiểm tra đường line màu đen và điều chỉnh hướng nếu cần
            # if self.sensors.detect_line_black():
            #     print("Phát hiện đường line màu đen, đang điều chỉnh hướng...")
            #     self.adjust_direction()
            #     continue
            
            # Tránh chướng ngại vật
            self.avoid_obstacles()  

            # # Kiểm tra pin và quay về trạm sạc nếu cần
            # if self.should_return_to_charge_station():
            #     print("Pin yếu, đang quay về trạm sạc...")
            #     self.return_to_charge_station()
            #     continue
            
            # # Nhiệm vụ chính: tưới cây
            # if not self.is_watering and self.has_plants_to_water():
            #     self.water_plants()
            #     continue

            # # Tìm kiếm và xử lý vật thể khi không có nhiệm vụ khác
            # detected_object = self.find_objects()
            # if detected_object:
            #     print("Phát hiện vật thể:", detected_object)
            #     self.handle_detected_object(detected_object)

            # # Thêm một khoảng thời gian nghỉ để giảm tải CPU
            # sleep(0.05)  # Giúp vòng lặp không chiếm quá nhiều tài nguyên
    
    
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

    def is_aligned_with_object(self, obj):
        """
        Kiểm tra xem robot đã thẳng hàng với vật thể chưa.
        """
        # Lấy vị trí của vật thể (giả sử có thuộc tính x, y trong obj)
        object_x = obj.get("x")  # Tọa độ x của vật thể
        object_y = obj.get("y")  # Tọa độ y của vật thể

        # Tính toán xem robot có thẳng hàng với vật thể không
        # Đây là một ví dụ đơn giản, bạn có thể thay đổi logic tùy theo cách tính toán của bạn
        # Ví dụ: kiểm tra nếu object_x gần với 0 (trung tâm)
        return abs(object_x) < 5  # Giả sử robot nằm ở x=0, điều chỉnh khoảng cách này nếu cần


    def return_to_charge_station(self):
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