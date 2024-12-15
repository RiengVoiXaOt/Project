from config.gpio_config import MOTORS  # Nhập cấu hình động cơ từ mô-đun gpio_config

class Motors:
    def __init__(self):
        self.motors = MOTORS  # Khởi tạo đối tượng Motors với cấu hình động cơ

    def set_direction(self, motor, forward):
        # Thiết lập hướng quay cho động cơ
        self.motors[motor]["forward"].value = forward  # Gán giá trị cho chân điều khiển quay tới
        self.motors[motor]["backward"].value = not forward  # Gán giá trị cho chân điều khiển quay lùi

    def set_speed(self, motor, speed):
        # Thiết lập tốc độ cho động cơ
        self.motors[motor]["en"].value = max(0, min(speed, 1))  # Giới hạn tốc độ từ 0 đến 1

    def stop_all(self):
        # Dừng tất cả các động cơ
        for motor in self.motors:
            self.set_speed(motor, 0)  # Đặt tốc độ của từng động cơ về 0
