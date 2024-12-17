from time import sleep, time
from src.hardware.relay import RelayControl
from src.hardware.servos import ServoControl
from src.hardware.motors import Motors
from src.hardware.ultrasonic import UltrasonicSensors
from src.vision.color_detection import color_detection_loop
from src.vision.object_detection import object_detection_loop
from src.utils.control_utils import getch, direction, set_motors_direction
from src.hardware.kinematic import Kinematic
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
    MAX_SPEED = 10  # Tốc độ tối đa

    def __init__(self, theta=None):
        # Khởi tạo các thành phần phần cứng của robot
        self.relay_control = RelayControl(5)  # Điều khiển relay cho tưới cây
        self.servo_control = ServoControl()  # Điều khiển servo
        self.motors = Motors()  # Điều khiển động cơ
        self.ultrasonic_sensors = UltrasonicSensors()  # Cảm biến siêu âm
        self.manual_mode = False  # Biến kiểm tra chế độ thủ công
        self.is_watering = False  # Biến kiểm tra trạng thái tưới cây
        self.charge_station_found = False  # Biến kiểm tra trạng thái tìm thấy trạm sạc
        if theta is None:
            theta = 0  # Góc quay mặc định
        self.speed = 0.04309596457  # Tốc độ tối thiểu (10% tốc độ tối đa)
        self.vx = self.MAX_SPEED * self.speed  # Tốc độ theo hướng x
        self.vy = self.MAX_SPEED * self.speed  # Tốc độ theo hướng y
        self.theta = theta  # Góc quay
        self.robot = Kinematic(0, 0, 0, 0)  # Khởi tạo đối tượng kinematic cho robot
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
            print("Enter command (w/a/s/d/q/e/z/x/1/2/3 to move, r to toggle relay, u to move servo up, i to move servo down, p to quit): ")
            command = getch()  # Nhận lệnh từ bàn phím
            if command == 'p':
                self.servo_control.reset_servos()  # Đặt lại servo
                print("Exiting manual control.")
                return
            elif command in ['+', '=']:
                self.n = min(self.n + 1, 10)  # Tăng tốc độ lên tối đa 100%
                self.vx = self.n * self.speed
                self.vy = self.n * self.speed
                print(f"Tốc độ tăng lên {self.n * 10}%")
            elif command in ['-', '_']:
                self.n = max(self.n - 1, 0)  # Giảm tốc độ xuống tối thiểu 0%
                self.vx = self.n * self.speed
                self.vy = self.n * self.speed
                print(f"Tốc độ giảm xuống {self.n * 10}%")
            elif command in direction:
                current_direction = direction[command]  # Lấy hướng di chuyển từ lệnh
                print("Direction: " + current_direction)
                set_motors_direction(self.robot, current_direction, self.vx, self.vy, self.theta)  # Thiết lập hướng động cơ
            elif command == 'r':
                self.relay_control.run_relay_for_duration()  # Bật relay tưới cây
            elif command == 'u':
                self.servo_control.move_servo_up("bottom")  # Di chuyển servo lên
            elif command == 'i':
                self.servo_control.move_servo_down("bottom")  # Di chuyển servo xuống

    def automatic_mode(self):
        """Chế độ tự động cho robot thực hiện nhiệm vụ."""
        print("Chế độ tự động đang chạy...")

        while not self.manual_mode:  # Vòng lặp chạy khi ở chế độ tự động
            # Kiểm tra đường line màu đen và điều chỉnh hướng nếu cần
            if self.sensors.detect_line_black():
                print("Phát hiện đường line màu đen, đang điều chỉnh hướng...")
                self.adjust_direction()
                continue
            
            # Tránh chướng ngại vật
            self.avoid_obstacles()  

            # Kiểm tra pin và quay về trạm sạc nếu cần
            if self.should_return_to_charge_station():
                print("Pin yếu, đang quay về trạm sạc...")
                self.return_to_charge_station()
                continue
            
            # Nhiệm vụ chính: tưới cây
            if not self.is_watering and self.has_plants_to_water():
                self.water_plants()
                continue

            # Tìm kiếm và xử lý vật thể khi không có nhiệm vụ khác
            detected_object = self.find_objects()
            if detected_object:
                print("Phát hiện vật thể:", detected_object)
                self.handle_detected_object(detected_object)

            # Thêm một khoảng thời gian nghỉ để giảm tải CPU
            sleep(0.05)  # Giúp vòng lặp không chiếm quá nhiều tài nguyên

                
    def avoid_obstacles(self):
        """Tránh chướng ngại vật bằng cách sử dụng cảm biến siêu âm."""
        # Đọc khoảng cách từ cảm biến
        front_distance = self.ultrasonic_sensors.get_distance("front")
        left_distance = self.ultrasonic_sensors.get_distance("left")
        right_distance = self.ultrasonic_sensors.get_distance("right")

        # Kiểm tra dữ liệu cảm biến
        if None in (front_distance, left_distance, right_distance):
            print("Lỗi cảm biến siêu âm: Không nhận được dữ liệu.")
            self.stop_robot()
            return

        # Kiểm tra và lưu lịch sử khoảng cách
        self.update_distance_history(front_distance, left_distance, right_distance)

        # Xác định trạng thái cảm biến
        f_state = front_distance < self.SAFE_DISTANCE
        l_state = left_distance < self.SAFE_DISTANCE
        r_state = right_distance < self.SAFE_DISTANCE

        # Tránh vật cản
        if not f_state:
            # Không có vật cản phía trước
            self.move_forward(front_distance)  # Di chuyển về phía trước
        else:
            # Có vật cản phía trước
            if front_distance <= self.CRITICAL_DISTANCE:
                self.move_backward()  # Lùi lại nếu gần vật cản
                return

            self.handle_side_obstacles(l_state, r_state, left_distance, right_distance)  # Xử lý vật cản bên

    def update_distance_history(self, front, left, right):
        """Lưu lịch sử khoảng cách và giới hạn kích thước."""
        for key, value in zip(['front', 'left', 'right'], [front, left, right]):
            self.distance_history[key].append(value)  # Thêm khoảng cách vào lịch sử
            if len(self.distance_history[key]) > self.MAX_HISTORY:
                self.distance_history[key].pop(0)  # Giới hạn kích thước lịch sử

    def move_forward(self, front_distance):
        """Đi thẳng, giảm tốc khi gần vật cản."""
        speed_factor = max(0.5, front_distance / self.SAFE_DISTANCE)  # Giảm tốc khi gần vật cản
        self.vx = self.MAX_SPEED * speed_factor  # Cập nhật tốc độ
        self.vy = self.MAX_SPEED * speed_factor
        set_motors_direction(self.robot, "go_forward", self.vx, self.vy, 0)  # Thiết lập hướng di chuyển
        self.state = "moving forward"  # Cập nhật trạng thái
        print(f"Đi thẳng với tốc độ: {self.vx:.2f}")

    def move_backward(self):
        """Lùi lại để tránh vật cản."""
        self.stop_robot()  # Dừng robot
        set_motors_direction(self.robot, "go_backward", self.vx, self.vy, 0)  # Thiết lập hướng lùi
        self.state = "moving backward"  # Cập nhật trạng thái
        print("Lùi lại để tránh vật cản")

    def stop_robot(self):
        """Dừng robot."""
        set_motors_direction(self.robot, "stop", 0, 0, 0)  # Dừng tất cả động cơ
        self.state = "stopped"  # Cập nhật trạng thái
        print("Robot dừng lại")

    def handle_side_obstacles(self, l_state, r_state, left_distance, right_distance):
        """Xử lý vật cản ở hai bên robot."""
        if not l_state and not r_state:
            # Cả hai bên đều không có vật cản
            if right_distance > left_distance:
                self.rotate_robot('rotate_right')  # Quay phải nếu bên phải có khoảng cách xa hơn
            else:
                self.rotate_robot('rotate_left')  # Quay trái nếu bên trái có khoảng cách xa hơn
        elif l_state and not r_state:
            self.rotate_robot('rotate_right')  # Quay phải nếu bên trái có vật cản
        elif r_state and not l_state:
            self.rotate_robot('rotate_left')  # Quay trái nếu bên phải có vật cản
        elif l_state and r_state:
            # Cả hai bên đều có vật cản
            if right_distance > left_distance:
                self.rotate_robot('rotate_right')  # Quay phải nếu bên phải có khoảng cách xa hơn
            else:
                self.rotate_robot('rotate_left')  # Quay trái nếu bên trái có khoảng cách xa hơn
            print("Cả hai bên đều có vật cản, chọn hướng quay.")

    def rotate_robot(self, direction, angle=90):
        """Quay robot sang trái hoặc phải."""
        max_rotate_time = angle / 90 * 3  # Thời gian xoay tương ứng với góc
        rotate_start_time = time()

        while time() - rotate_start_time < max_rotate_time:
            front_distance = self.ultrasonic_sensors.get_distance("front")  # Đọc khoảng cách phía trước
            if front_distance >= 1.5 * self.SAFE_DISTANCE:
                self.stop_robot()  # Dừng robot nếu khoảng cách phía trước an toàn
                break
            set_motors_direction(self.robot, direction, self.vx, self.vy, 0)  # Thiết lập hướng quay
            sleep(0.1)  # Thời gian nghỉ giữa các lần đọc
        self.stop_robot()  # Dừng robot sau khi quay
        print(f"Robot đã quay sang {'phải' if direction == 'rotate_right' else 'trái'}.")


    def find_objects(self):
        """Tìm kiếm vật thể và tương tác với nó."""
        object_found = object_detection_loop()  # Gọi hàm để tìm kiếm vật thể
        if object_found:
            print("Phát hiện vật thể!")
            # Logic tương tác với vật thể
            self.relay_control.toggle_relay(True)  # Bật relay khi phát hiện vật thể
            sleep(2)  # Giữ relay bật trong 2 giây
            self.relay_control.toggle_relay(False)  # Tắt relay
            # Có thể thêm logic khác để xử lý vật thể


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