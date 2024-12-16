from gpiozero import AngularServo # type: ignore
from src.config.gpio_config import servo_1, servo_2  # Nhập cấu hình từ gpio_config

class ServoControl:
    def __init__(self):
        """
        Khởi tạo hai servo trên và dưới từ cấu hình GPIO sử dụng AngularServo.
        """
        try:
            self.servo_top = servo_2  # Servo trên
            self.servo_bottom = servo_1  # Servo dưới
            self.servo_top.angle = 110  # Thiết lập góc mặc định cho servo trên
            self.servo_bottom.angle = 90  # Thiết lập góc mặc định cho servo dưới
        except Exception as e:
            print(f"Error initializing servos: {e}")
            self.servo_top = None
            self.servo_bottom = None

    def reset_servos(self):
        """
        Đặt lại cả hai servo về vị trí mặc định (0 độ).
        """
        try:
            if self.servo_top:
                self.servo_top.angle = 110
                print("Reset top servo to default position (110 degrees).")
            if self.servo_bottom:
                self.servo_bottom.angle = 90
                print("Reset bottom servo to default position (90 degrees).")
        except Exception as e:
            print(f"Error resetting servos: {e}")
            
    def move_servo_up(self, servo_name):
        """
        Di chuyển servo chỉ định lên (tăng góc thêm 10 độ).
        :param servo_name: 'top' hoặc 'bottom'
        """
        try:
            if servo_name == "top" and self.servo_top:
                current_angle = self.servo_top.angle or 0  # Đảm bảo góc hiện tại là hợp lệ
                new_angle = min(current_angle + 10, 180)  # Giới hạn góc tối đa là 180 độ
                self.servo_top.angle = new_angle
                print(f"Moved top servo up to {new_angle} degrees.")
            elif servo_name == "bottom" and self.servo_bottom:
                current_angle = self.servo_bottom.angle or 0
                new_angle = min(current_angle + 10, 180)
                self.servo_bottom.angle = new_angle
                print(f"Moved bottom servo up to {new_angle} degrees.")
            else:
                print(f"Invalid servo name or servo is not initialized.")
        except Exception as e:
            print(f"Error moving servo up: {e}")

    def move_servo_down(self, servo_name):
        """
        Di chuyển servo chỉ định xuống (giảm góc thêm 10 độ).
        :param servo_name: 'top' hoặc 'bottom'
        """
        try:
            if servo_name == "top" and self.servo_top:
                current_angle = self.servo_top.angle or 0  # Đảm bảo góc hiện tại là hợp lệ
                new_angle = max(current_angle - 10, 0)  # Giới hạn góc tối thiểu là 0 độ
                self.servo_top.angle = new_angle
                print(f"Moved top servo down to {new_angle} degrees.")
            elif servo_name == "bottom" and self.servo_bottom:
                current_angle = self.servo_bottom.angle or 0
                new_angle = max(current_angle - 10, 0)
                self.servo_bottom.angle = new_angle
                print(f"Moved bottom servo down to {new_angle} degrees.")
            else:
                print(f"Invalid servo name or servo is not initialized.")
        except Exception as e:
            print(f"Error moving servo down: {e}")
