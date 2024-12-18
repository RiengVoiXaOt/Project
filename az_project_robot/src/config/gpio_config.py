from gpiozero import DigitalOutputDevice, PWMOutputDevice, DistanceSensor# type: ignore
from adafruit_servokit import ServoKit

pca = ServoKit(channels=16)
# === Motor Configuration ===
# Cấu hình các động cơ với các chân GPIO tương ứng
MOTORS = {
    "motor_1": {
        "forward": DigitalOutputDevice(5),   # Chân GPIO cho động cơ 1 quay tới
        "backward": DigitalOutputDevice(6),  # Chân GPIO cho động cơ 1 quay lùi
        "enable": PWMOutputDevice(18)          # Chân GPIO để điều khiển tốc độ động cơ 1
    },
    "motor_2": {
        "forward": DigitalOutputDevice(7),   # Chân GPIO cho động cơ 2 quay tới
        "backward": DigitalOutputDevice(8),  # Chân GPIO cho động cơ 2 quay lùi
        "enable": PWMOutputDevice(19)          # Chân GPIO để điều khiển tốc độ động cơ 2
    },
    "motor_3": {
        "forward": DigitalOutputDevice(16),  # Chân GPIO cho động cơ 3 quay tới
        "backward": DigitalOutputDevice(25), # Chân GPIO cho động cơ 3 quay lùi
        "enable": PWMOutputDevice(12)         # Chân GPIO để điều khiển tốc độ động cơ 3
    },
    "motor_4": {
        "forward": DigitalOutputDevice(10),  # Chân GPIO cho động cơ 4 quay tới
        "backward": DigitalOutputDevice(26), # Chân GPIO cho động cơ 4 quay lùi
        "enable": PWMOutputDevice(13)         # Chân GPIO để điều khiển tốc độ động cơ 4
    }
}

# === Servo Configuration ===
# Cấu hình các servo với chân GPIO tương ứng
servo_1 = pca.servo[0]
servo_2 = pca.servo[11]   # Servo duoi

# === Ultrasonic Sensor Configuration ===
# Cấu hình các cảm biến siêu âm với chân GPIO cho echo và trigger
# === Ultrasonic Sensor Configuration ===
F_SENSOR = DistanceSensor(echo=11, trigger=9)    # Cảm biến phía trước
L_SENSOR = DistanceSensor(echo=27, trigger=17)   # Cảm biến bên trái
R_SENSOR = DistanceSensor(echo=24, trigger=23)   # Cảm biến bên phải

# === Relay Configuration ===
# Cấu hình relay với chân GPIO tương ứng
RELAY = DigitalOutputDevice(4)  # Relay để kiểm soát các thiết bị khác

# === Water Level Sensor Configuration ===
# Cấu hình cảm biến mức nước với chân GPIO tương ứng
WATER_LEVEL_SENSOR = DigitalOutputDevice(22)  # Cảm biến mức nước
