import smbus
import time

DEFAULT_ANGLE = 60  # Góc mặc định
MIN_ANGLE = 0  # Giới hạn góc nhỏ nhất
MAX_ANGLE = 120
# === PCA9685 Configuration ===
# Cấu hình bus I2C và địa chỉ PCA9685
bus = smbus.SMBus(4)  # Sử dụng bus 4
pca9685_address = 0x40  # Địa chỉ của PCA9685

# Đặt tần số PWM (khuyến nghị: 50Hz cho servo)
def set_pwm_freq(bus, address, freq_hz):
    prescale_val = int(25000000.0 / (4096 * freq_hz) - 1)  # Tính giá trị prescale
    old_mode = bus.read_byte_data(address, 0x00)  # Đọc chế độ cũ
    new_mode = (old_mode & 0x7F) | 0x10  # Chuyển sang chế độ sleep
    bus.write_byte_data(address, 0x00, new_mode)
    bus.write_byte_data(address, 0xFE, prescale_val)  # Đặt giá trị prescale
    bus.write_byte_data(address, 0x00, old_mode)
    time.sleep(0.005)  # Đợi chế độ sleep kết thúc
    bus.write_byte_data(address, 0x00, old_mode | 0x80)  # Chuyển sang chế độ normal

# Thiết lập xung PWM cho một kênh cụ thể
def set_pwm(bus, address, channel, on, off):
    bus.write_byte_data(address, 0x06 + 4 * channel, on & 0xFF)  # Đặt giá trị thấp (LOW byte)
    bus.write_byte_data(address, 0x07 + 4 * channel, on >> 8)    # Đặt giá trị cao (HIGH byte)
    bus.write_byte_data(address, 0x08 + 4 * channel, off & 0xFF)  # Đặt giá trị thấp (LOW byte)
    bus.write_byte_data(address, 0x09 + 4 * channel, off >> 8)    # Đặt giá trị cao (HIGH byte)

# Hàm chuyển đổi góc servo (0-180 độ) sang giá trị xung (on, off)
def angle_to_pwm(angle, pulse_min=150, pulse_max=600):
    pulse_length = pulse_min + (angle / 180.0) * (pulse_max - pulse_min)
    return int(pulse_length)
# === ServoControl Class ===
class ServoControl:
    DEFAULT_ANGLE = 60  # Góc mặc định
    MIN_ANGLE = 0  # Giới hạn góc nhỏ nhất
    MAX_ANGLE = 120
    def __init__(self, channel):
        self.channel = channel  # Kênh của servo trên PCA9685
        self.angle = self.DEFAULT_ANGLE  # Góc ban đầu (60 độ)
        self.move_to_angle(self.angle)  # Đặt góc ban đầu

    @property
    def current_angle(self):
        return self.angle

    def move_up(self, step=10):
        """
        Di chuyển servo lên (tăng góc thêm `step` độ).
        """
        new_angle = min(self.angle + step, self.MAX_ANGLE)  # Giới hạn tối đa là 120 độ
        if new_angle != self.angle:
            self.move_to_angle(new_angle)
            print(f"Moved servo up to {self.angle} degrees.")
        else:
            print("Servo is already at the maximum angle.")

    def move_down(self, step=10):
        """
        Di chuyển servo xuống (giảm góc thêm `step` độ).
        """
        new_angle = max(self.angle - step, self.MIN_ANGLE)  # Giới hạn tối thiểu là 10 độ
        if new_angle != self.angle:
            self.move_to_angle(new_angle)
            print(f"Moved servo down to {self.angle} degrees.")
        else:
            print("Servo is already at the minimum angle.")

    def reset(self):
        """
        Đặt lại servo về vị trí mặc định (60 độ).
        """
        self.move_to_angle(self.DEFAULT_ANGLE)  # Đặt lại góc về 60 độ

    def move_to_angle(self, target_angle):
        """
        Di chuyển servo đến một góc cụ thể.
        """
        if self.MIN_ANGLE <= target_angle <= self.MAX_ANGLE:
            self.angle = target_angle
            pwm_val = angle_to_pwm(self.angle)
            set_pwm(bus, pca9685_address, self.channel, 0, pwm_val)  # Gửi tín hiệu PWM tới kênh
            print(f"Moved servo to {self.angle} degrees.")
        else:
            print("Angle must be between 0 and 120 degrees.")
    def servo_move_to_search(self):
        """
        Di chuyển servo từ góc nhỏ nhất đến lớn nhất để tìm vật,
        dừng lại ngay khi phát hiện được vật thông qua hàm callback.
        
        detection_callback: Hàm kiểm tra vật được phát hiện (trả về True nếu phát hiện).
        """
        for angle in range(self.MIN_ANGLE, self.MAX_ANGLE + 1, 10):
            self.move_to_angle(angle)  # Di chuyển tới góc hiện tại
            time.sleep(0.5)  # Dừng lại tại mỗi góc để kiểm tra
            print(f"Searching at {angle} degrees.")
        

# Khởi động PCA9685
bus.write_byte_data(pca9685_address, 0x00, 0x01)  # Reset PCA9685
set_pwm_freq(bus, pca9685_address, 50)  # Thiết lập tần số 50Hz cho servo