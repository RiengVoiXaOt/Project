class ServoControl:
    def __init__(self, servo_object):
        self.servo_object = servo_object  # Đối tượng servo từ gpio_config
        self.angle = 90  # Khởi tạo góc cho servo

        # Cập nhật góc ban đầu cho servo
        self.servo_object.angle = self.angle

    def move_up(self, step=10):
        """
        Di chuyển servo lên (tăng góc thêm 10 độ).
        """
        new_angle = min(self.angle + step, 180)  # Giới hạn tối đa là 180 độ
        if new_angle != self.angle:
            self.angle = new_angle
            self.servo_object.angle = self.angle  # Cập nhật góc của đối tượng servo
            print(f"Moved {self.servo_object} servo up to {self.angle} degrees.")
        else:
            print(f"{self.servo_object} servo is already at the maximum angle.")

    def move_down(self, step=10):
        """
        Di chuyển servo xuống (giảm góc thêm 10 độ).
        """
        new_angle = max(self.angle - step, 0)  # Giới hạn tối thiểu là 0 độ
        if new_angle != self.angle:
            self.angle = new_angle
            self.servo_object.angle = self.angle  # Cập nhật góc của đối tượng servo
            print(f"Moved {self.servo_object} servo down to {self.angle} degrees.")
        else:
            print(f"{self.servo_object} servo is already at the minimum angle.")

    def reset(self):
        """
        Đặt lại servo về vị trí mặc định (90 độ).
        """
        self.angle = 90
        self.servo_object.angle = self.angle  # Cập nhật góc của đối tượng servo
        print(f"Reset {self.servo_object} servo to default position (90 degrees).")