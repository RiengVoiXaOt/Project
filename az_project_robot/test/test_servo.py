import smbus
import time

# Cấu hình bus I2C và địa chỉ PCA9685
bus4 = smbus.SMBus(4)  # Sử dụng bus 4
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

# Hàm điều khiển servo
def move_servo(bus, address, channel, angle):
    pwm_val = angle_to_pwm(angle)
    set_pwm(bus, address, channel, 0, pwm_val)

# Khởi động PCA9685
bus4.write_byte_data(pca9685_address, 0x00, 0x01)  # Reset PCA9685
set_pwm_freq(bus4, pca9685_address, 50)  # Thiết lập tần số 50Hz cho servo

# Di chuyển servo qua lại
if __name__ == "__main__":
    servo_channel = 1  # Kênh servo 1 (kênh 0)
    try:
        while True:
            print("Moving servo from 10 to 120 degrees")
            for angle in range(10, 121, 2):  # Tăng từ 10 đến 120 độ, mỗi bước 2 độ
                move_servo(bus4, pca9685_address, servo_channel, angle)
                time.sleep(0.05)  # Nghỉ 50ms giữa mỗi bước

            print("Moving servo from 120 to 10 degrees")
            for angle in range(120, 9, -2):  # Giảm từ 120 về 10 độ, mỗi bước 2 độ
                move_servo(bus4, pca9685_address, servo_channel, angle)
                time.sleep(0.05)  # Nghỉ 50ms giữa mỗi bước
    except KeyboardInterrupt:
        print("Exiting program")
