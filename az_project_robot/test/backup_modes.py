from time import sleep
from hardware.relay import RelayControl
from hardware.servos import ServoControl
from hardware.motors import Motors
from hardware.ultrasonic import UltrasonicSensors
from vision.color_detection import color_detection_loop
from vision.object_detection import object_detection_loop
from hardware.kinematic import Kinematic

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
    def __init__(self):
        self.relay_control = RelayControl()
        self.servo_control = ServoControl()
        self.motors = Motors()
        self.ultrasonic_sensors = UltrasonicSensors()
        self.manual_mode = False
        self.is_watering = False
        self.charge_station_found = False

    def switch_mode(self):
        self.manual_mode = not self.manual_mode
        mode = "manual" if self.manual_mode else "automatic"
        print(f"Switched to {mode} mode.")

    def manual_control(self):
        while self.manual_mode:
            command = input("Enter command (w/a/s/d to move, r to toggle relay, u to move servo up, d to move servo down, q to quit): ")
            if command == 'w':
                self.move_forward()
            elif command == 's':
                self.move_backward()
            elif command == 'a':
                self.turn_left()
            elif command == 'd':
                self.turn_right()
            elif command == 'r':
                self.toggle_relay()
            elif command == 'u':
                self.servo_control.move_servo("top", 180)
            elif command == 'd':
                self.servo_control.move_servo("bottom", 0)
            elif command == 'q':
                break
            sleep(0.1)

    def move_forward(self):
        self.motors.set_direction("motor_1", True)
        self.motors.set_direction("motor_2", True)
        self.motors.set_speed("motor_1", 1)
        self.motors.set_speed("motor_2", 1)

    def move_backward(self):
        self.motors.set_direction("motor_1", False)
        self.motors.set_direction("motor_2", False)
        self.motors.set_speed("motor_1", 1)
        self.motors.set_speed("motor_2", 1)

    def turn_left(self):
        self.motors.set_direction("motor_3", True)
        self.motors.set_speed("motor_3", 1)

    def turn_right(self):
        self.motors.set_direction("motor_4", True)
        self.motors.set_speed("motor_4", 1)

    def toggle_relay(self):
        self.relay_control.toggle_relay(True)
        sleep(2)  # Relay bật trong 2 giây
        self.relay_control.toggle_relay(False)

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
        distance_front = self.ultrasonic_sensors.get_distance("front")
        distance_left = self.ultrasonic_sensors.get_distance("left")
        distance_right = self.ultrasonic_sensors.get_distance("right")

        if distance_front < 20:  # Nếu có vật cản phía trước
            print("Obstacle detected in front!")
            # Logic tránh chướng ngại vật
            if distance_left > distance_right:
                self.turn_right()  # Quay phải
            else:
                self.turn_left()  # Quay trái
            sleep(1)  # Chờ một chút trước khi kiểm tra lại

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

