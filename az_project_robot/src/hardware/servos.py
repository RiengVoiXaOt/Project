class ServoControl:
    def __init__(self, servo_object):
        if not hasattr(servo_object, 'angle'):
            raise ValueError("Provided object is not a valid servo.")
        
        self.servo_object = servo_object  # Đối tượng servo từ gpio_config
        self.angle = 90  # Khởi tạo góc cho servo

        # Cập nhật góc ban đầu cho servo
        self.servo_object.angle = self.angle

    @property
    def current_angle(self):
        return self.angle

    def move_up(self, step=10):
        """
        Di chuyển servo lên (tăng góc thêm 10 độ).
        """
        new_angle = min(self.angle + step, 180)  # Giới hạn tối đa là 180 độ
        if new_angle != self.angle:
            self.angle = new_angle
            self.servo_object.angle = self.angle  # Cập nhật góc của đối tượng servo
            print(f"Moved servo up to {self.angle} degrees.")
        else:
            print("Servo is already at the maximum angle.")

    def move_down(self, step=10):
        """
        Di chuyển servo xuống (giảm góc thêm 10 độ).
        """
        new_angle = max(self.angle - step, 0)  # Giới hạn tối thiểu là 0 độ
        if new_angle != self.angle:
            self.angle = new_angle
            self.servo_object.angle = self.angle  # Cập nhật góc của đối tượng servo
            print(f"Moved servo down to {self.angle} degrees.")
        else:
            print("Servo is already at the minimum angle.")

    def reset(self):
        """
        Đặt lại servo về vị trí mặc định (0 độ).
        """
        self.move_to_angle(0)  # Sử dụng phương thức move_to_angle để đặt lại

    def move_to_angle(self, target_angle):
        """
        Di chuyển servo đến một góc cụ thể.
        """
        if 0 <= target_angle <= 180:
            self.angle = target_angle
            self.servo_object.angle = self.angle
            print(f"Moved servo to {self.angle} degrees.")
        else:
            print("Angle must be between 0 and 180 degrees.")
