from gpiozero import AngularServo # type: ignore
from config.gpio_config import SERVOS  # Nhập cấu hình từ gpio_config

class ServoControl:
    def __init__(self):
        """
        Khởi tạo hai servo trên và dưới từ cấu hình GPIO sử dụng AngularServo.
        """
        try:
            self.servo_top = AngularServo(SERVOS["servo_2"].pin, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)  # Servo trên
            self.servo_bottom = AngularServo(SERVOS["servo_1"].pin, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.4/1000)  # Servo dưới
        except Exception as e:
            print(f"Error initializing servos: {e}")
            self.servo_top = None
            self.servo_bottom = None

    def move_servo(self, servo_name, angle):
        """
        Di chuyển servo đến góc chỉ định.
        :param servo_name: 'top' hoặc 'bottom' để chọn servo
        :param angle: Giá trị góc (0 đến 180 độ)
        """
        if angle < 0 or angle > 180:
            print("Angle must be between 0 and 180 degrees.")
            return

        try:
            if servo_name == "top" and self.servo_top:
                self.servo_top.angle = angle
                print(f"Moved top servo to angle {angle} degrees.")
            elif servo_name == "bottom" and self.servo_bottom:
                self.servo_bottom.angle = angle
                print(f"Moved bottom servo to angle {angle} degrees.")
            else:
                print("Invalid servo name or servo not initialized.")
        except Exception as e:
            print(f"Error moving {servo_name} servo: {e}")

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
