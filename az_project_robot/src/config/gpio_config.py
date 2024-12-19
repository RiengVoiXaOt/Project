from gpiozero import DigitalOutputDevice, PWMOutputDevice, DistanceSensor  # type: ignore
from adafruit_servokit import ServoKit
import board
import busio

# === Motor Configuration ===
# Cấu hình các động cơ với các chân GPIO tương ứng
MOTORS = {
    "motor_1": {
        "forward": DigitalOutputDevice(6),   # Chân GPIO cho động cơ 1 quay tới
        "backward": DigitalOutputDevice(26),  # Chân GPIO cho động cơ 1 quay lùi
        "enable": PWMOutputDevice(13)          # Chân GPIO để điều khiển tốc độ động cơ 1
    },
    "motor_2": {
        "forward": DigitalOutputDevice(16),  # Chân GPIO cho động cơ 2 quay tới
        "backward": DigitalOutputDevice(7), # Chân GPIO cho động cơ 2 quay lùi
        "enable": PWMOutputDevice(12)         # Chân GPIO để điều khiển tốc độ động cơ 3
    
    },
    "motor_3": {
        "forward": DigitalOutputDevice(24),   # Chân GPIO cho động cơ 3 quay tới
        "backward": DigitalOutputDevice(23),  # Chân GPIO cho động cơ 3 quay lùi
        "enable": PWMOutputDevice(18)          # Chân GPIO để điều khiển tốc độ động cơ 2
    },
    "motor_4": {
        "forward": DigitalOutputDevice(20),  # Chân GPIO cho động cơ 4 quay tới
        "backward": DigitalOutputDevice(21), # Chân GPIO cho động cơ 4 quay lùi
        "enable": PWMOutputDevice(19)         # Chân GPIO để điều khiển tốc độ động cơ 4
    }
}

# === Servo Configuration ===
# Cấu hình các servo với PCA9685 (thông qua ServoKit)
i2c_bus = busio.I2C(board.SCL, board.SDA)  # Sử dụng GPIO 2 (SDA) và GPIO 3 (SCL) cho I2C
kit = ServoKit(channels=16, i2c=i2c_bus, address=0x40)

# === Ultrasonic Sensor Configuration ===
# Cấu hình các cảm biến siêu âm với chân GPIO cho echo và trigger
F_SENSOR = DistanceSensor(echo=9, trigger=10)    # Cảm biến phía trước
L_SENSOR = DistanceSensor(echo=17, trigger=4)    # Cảm biến bên trái
R_SENSOR = DistanceSensor(echo=22, trigger=27)   # Cảm biến bên phải

# === Relay Configuration ===
# Cấu hình relay với chân GPIO tương ứng
RELAY = DigitalOutputDevice(5)  # Relay để kiểm soát các thiết bị khác

# === Water Level Sensor Configuration ===
# Mức nước đo bằng GPIO 11
WATER_LEVEL_SENSOR = DigitalOutputDevice(11)
