from src.config.gpio_config import L_SENSOR, F_SENSOR, R_SENSOR # Nhập các cảm biến siêu âm từ mô-đun gpio_config

class UltrasonicSensors:
    def __init__(self):
        # Khởi tạo từ điển chứa các cảm biến siêu âm
        self.sensors = {
            "front": F_SENSOR,  # Cảm biến phía trước
            "left": L_SENSOR,   # Cảm biến bên trái
            "right": R_SENSOR,  # Cảm biến bên phải
        }

    def get_distance(self, direction):
        # Lấy khoảng cách từ cảm biến theo hướng chỉ định
        sensor = self.sensors.get(direction)  # Lấy cảm biến dựa trên hướng
        return sensor.distance * 100 if sensor else None  # Trả về khoảng cách (cm) hoặc None nếu không tìm thấy cảm biến

    def print_distances(self):
        """In ra khoảng cách từ ba cảm biến."""
        front_distance = self.get_distance("front")
        left_distance = self.get_distance("left")
        right_distance = self.get_distance("right")

        print(f"Khoảng cách cảm biến phía trước: {front_distance} cm")
        print(f"Khoảng cách cảm biến bên trái: {left_distance} cm")
        print(f"Khoảng cách cảm biến bên phải: {right_distance} cm")
