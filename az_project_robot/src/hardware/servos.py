from src.config.gpio_config import servo_1, servo_2  # Nhập cấu hình từ gpio_config

class ServoControl:
    def __init__(self):
        """
        Khởi tạo góc cho servo trên và dưới.
        """
        self.servo_top = servo_2  # Servo trên
        self.servo_bottom = servo_1  # Servo dưới
        self.servo_top.angle = 90  # Góc khởi tạo cho servo trên
        self.servo_bottom.angle = 90  # Góc khởi tạo cho servo dưới

    def reset_servos(self):
        """
        Đặt lại cả hai servo về vị trí mặc định (90 độ).
        """
        self.servo_top.angle = 90
        self.servo_bottom.angle = 90
        print("Reset top servo to default position (90 degrees).")
        print("Reset bottom servo to default position (90 degrees).")

    def move_servo_up(self, servo_name):
        """
        Di chuyển servo chỉ định lên (tăng góc thêm 10 độ).
        :param servo_name: 'top' hoặc 'bottom'
        """
        if servo_name == "top":
            current_angle = self.servo_top.angle
            new_angle = min(current_angle + 10, 180)  # Giới hạn tối đa là 180 độ
            if new_angle != current_angle:
                self.servo_top.angle = new_angle
                print(f"Moved top servo up to {new_angle} degrees.")
            else:
                print("Top servo is already at the maximum angle.")
        elif servo_name == "bottom":
            current_angle = self.servo_bottom.angle
            new_angle = min(current_angle + 10, 180)
            if new_angle != current_angle:
                self.servo_bottom.angle = new_angle
                print(f"Moved bottom servo up to {new_angle} degrees.")
            else:
                print("Bottom servo is already at the maximum angle.")

    def move_servo_down(self, servo_name):
        """
        Di chuyển servo chỉ định xuống (giảm góc thêm 10 độ).
        :param servo_name: 'top' hoặc 'bottom'
        """
        if servo_name == "top":
            current_angle = self.servo_top.angle
            new_angle = max(current_angle - 10, 0)  # Giới hạn tối thiểu là 0 độ
            if new_angle != current_angle:
                self.servo_top.angle = new_angle
                print(f"Moved top servo down to {new_angle} degrees.")
            else:
                print("Top servo is already at the minimum angle.")
        elif servo_name == "bottom":
            current_angle = self.servo_bottom.angle
            new_angle = max(current_angle - 10, 0)
            if new_angle != current_angle:
                self.servo_bottom.angle = new_angle
                print(f"Moved bottom servo down to {new_angle} degrees.")
            else:
                print("Bottom servo is already at the minimum angle.")