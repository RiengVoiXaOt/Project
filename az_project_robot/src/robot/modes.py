from time import sleep, time
from src.hardware.relay import RelayControl
from src.hardware.servos import ServoControl
from src.hardware.motors import Motors
from src.hardware.ultrasonic import UltrasonicSensors
from src.vision.color_detection import color_detection_loop
from src.vision.object_detection import object_detection_loop
from src.utils.control_utils import getch, direction, set_motors_direction
from src.hardware.servos import ServoControl
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
    def __init__(self, n = None, theta = None):
        self.relay_control = RelayControl(5)
        self.servo_control = ServoControl()
        self.motors = Motors()
        self.ultrasonic_sensors = UltrasonicSensors()
        self.manual_mode = False
        self.is_watering = False
        self.charge_station_found = False
        if n == None:
            n = 5
        if theta == None:
            theta = 0
        self.n = n  # Khởi tạo biến tốc độ
        self.speed = 0.04309596457 # 10% vận tốc của động cơ tương ứng với 0.1
        self.vx = n * self.speed
        self.vy = n * self.speed
        self.theta = theta
        self.robot = Kinematic(0, 0, 0, 0 )
        
    def switch_mode(self):
        self.manual_mode = not self.manual_mode
        mode = "manual" if self.manual_mode else "automatic"
        print(f"Switched to {mode} mode.")
        if self.manual_mode:
            self.motors.stop_all()  # Dừng tất cả động cơ khi chuyển sang chế độ thủ công

    def manual_control(self):
        while self.manual_mode:
            print("Enter command (w/a/s/d/q/e/z/x/1/2/3 to move, r to toggle relay, u to move servo up, i to move servo down, p to quit): ")
            command = getch()
            if command == 'p':
                self.servo_control.reset_servos()
                print("Exiting manual control.")
                return
            elif command in ['+', '=']:
                self.n = min(self.n + 1, 10)
                self.vx = self.n * self.speed
                self.vy = self.n * self.speed
                print(f"Tốc độ tăng lên {self.n * 10}%")
            elif command in ['-', '_']:
                self.n = max(self.n - 1, 0)
                self.vx = self.n * self.speed
                self.vy = self.n * self.speed
                print(f"Tốc độ giảm xuống {self.n * 10}%")
            elif command in direction:
                current_direction = direction[command]
                print("Direction: " + current_direction)
                set_motors_direction(self.robot, current_direction, self.vx, self.vy, self.theta)
            elif command == 'r':
                self.relay_control.run_relay_for_duration()
            elif command == 'u':
                self.servo_control.move_servo_up("bottom")
            elif command == 'i':
                self.servo_control.move_servo_down("bottom")

    def automatic_mode(self):
        while not self.manual_mode:
            print("Automatic mode running...")
            if not self.is_watering:
                self.water_plants()
            self.avoid_obstacles()
            self.find_objects()
            self.return_to_charge_station()

    def water_plants(self):
        print("Starting watering plants...")
        self.relay_control.toggle_relay(True)  # Bật relay để tưới
        sleep(10)  # Tưới trong 10 giây
        self.relay_control.toggle_relay(False)  # Tắt relay
        self.is_watering = False  # Đặt lại trạng thái tưới cây
        print("Finished watering plants.")

    def avoid_obstacles(self):
        """
        Hàm tránh vật cản dựa trên cảm biến siêu âm, sử dụng set_motors_direction.
        Cải tiến với logic phức tạp và lưu trữ dữ liệu lịch sử.
        """
        # Đọc khoảng cách từ cảm biến
        front_distance = self.ultrasonic_sensors.get_distance("front")
        left_distance = self.ultrasonic_sensors.get_distance("left")
        right_distance = self.ultrasonic_sensors.get_distance("right")

        # Kiểm tra nếu bất kỳ cảm biến nào trả về None
        if front_distance is None or left_distance is None or right_distance is None:
            print("Ultrasonic sensor error: Missing distance data.")
            set_motors_direction(self.robot, "stop", 0, 0, 0)
            return

        print(f"Distances - Front: {front_distance} cm, Left: {left_distance} cm, Right: {right_distance}")

        # Lưu trữ khoảng cách vào lịch sử
        if not hasattr(self, 'distance_history'):
            self.distance_history = {'front': [], 'left': [], 'right': []}
        
        self.distance_history['front'].append(front_distance)
        self.distance_history['left'].append(left_distance)
        self.distance_history['right'].append(right_distance)

        # Xác định trạng thái của cảm biến
        f_state = 1 if front_distance < 15 else 0
        l_state = 1 if left_distance < 15 else 0
        r_state = 1 if right_distance < 15 else 0

        if f_state == 0:
            # Không có chướng ngại phía trước
            if l_state == 0 and r_state == 0:
                set_motors_direction(self.robot, "go_forward", self.vx, self.vy, 0)
                print("Đi thẳng")
            elif r_state == 1:
                set_motors_direction(self.robot, "go_left", self.vx, self.vy, 0)
                print("Đi trái")
            elif l_state == 1:
                set_motors_direction(self.robot, "go_right", self.vx, self.vy, 0)
                print("Đi phải")
            else:
                # Điều chỉnh đi chéo nếu có vật cản bên trái hoặc phải
                direction = 'diagonal_up_right' if l_state == 1 else 'diagonal_up_left'
                set_motors_direction(self.robot, direction, self.vx, self.vy, 0)
                print(f"Đi chéo lên {'phải' if l_state == 1 else 'trái'}")

        else:
            # Có vật cản phía trước
            if front_distance <= 12:
                set_motors_direction(self.robot, "go_backward", self.vx, self.vy, 0)
                print("Lùi lại để tránh vật cản")
                return

            # Lựa chọn hướng xoay dựa trên trạng thái cảm biến
            if l_state == 0 and r_state == 0:
                # Nếu không có vật cản bên trái và bên phải
                if front_distance >= 10:
                    if right_distance > left_distance:
                        self.rotate_robot('rotate_right')
                    else:
                        self.rotate_robot('rotate_left')
            elif l_state == 1 and r_state == 0:
                self.rotate_robot('rotate_right')
            elif r_state == 1 and l_state == 0:
                self.rotate_robot('rotate_left')
            elif l_state == 1 and r_state == 1:
                # Nếu cả hai bên đều có vật cản, chọn bên nào có khoảng cách xa hơn để quay
                if right_distance > left_distance:
                    self.rotate_robot('rotate_right')
                else:
                    self.rotate_robot('rotate_left')

    def rotate_robot(self, direction):
        """
        Hàm xoay robot theo hướng chỉ định.
        """
        max_rotate_time = 3  # Giới hạn thời gian xoay (giây)
        rotate_start_time = time.time()

        while time.time() - rotate_start_time < max_rotate_time:
            front_distance = self.ultrasonic_sensors.get_distance("front")
            left_distance = self.ultrasonic_sensors.get_distance("left")
            right_distance = self.ultrasonic_sensors.get_distance("right")

            print(f"Khoảng cách: Trước {front_distance}, Trái {left_distance}, Phải {right_distance}")
            set_motors_direction(self.robot, direction, self.vx, self.vy, 0)
            print(f"Xoay {'phải' if direction == 'rotate_right' else 'trái'}")

            # Điều kiện thoát vòng lặp
            if front_distance >= 1.5 * 15:  # Nếu khoảng cách phía trước an toàn
                set_motors_direction(self.robot, 'stop', 0, 0, 0)
                print("Dừng xoay")
                break
        else:
            # Nếu vượt quá thời gian, chuyển sang phương án khác
            set_motors_direction(self.robot, 'go_backward', self.vx, self.vy, 0)
            print("Thay đổi hướng bằng cách lùi")



    def find_objects(self):
        # Gọi hàm để tìm kiếm vật thể
        object_found = object_detection_loop()
        if object_found:
            print("Object detected!")
            # Logic tương tác với vật thể
            self.relay_control.toggle_relay(True)  # Bật relay khi phát hiện vật thể
            sleep(2)  # Giữ relay bật trong 2 giây
            self.relay_control.toggle_relay(False)

    def return_to_charge_station(self):
        # Logic quay về trạm sạc
        if not self.charge_station_found:
            print("Searching for charge station...")
            line_found = color_detection_loop("red")  # Tìm đường line màu đỏ
            if line_found:
                print("Charge station line found! Following the line...")
                # Logic di chuyển theo đường line
                # Khi đến trạm sạc, đặt charge_station_found = True
                self.charge_station_found = True  # Giả sử đã tìm thấy trạm sạc

        # Nếu đã tìm thấy trạm sạc, di chuyển đến đó
        if self.charge_station_found:
            print("Moving to charge station...")
            # Logic di chuyển đến trạm sạc
            # Khi đến nơi, dừng động cơ và vào trạng thái nghỉ
            self.motors.stop_all()  # Dừng tất cả động cơ
            print("Robot is charging...")

