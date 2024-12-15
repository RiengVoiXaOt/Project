from gpiozero import DigitalOutputDevice, PWMOutputDevice, DistanceSensor, Servo # type: ignore

# === Motor Configuration ===
# Cấu hình các động cơ với các chân GPIO tương ứng
MOTORS = {
    "motor_1": {
        "forward": DigitalOutputDevice(5),   # Chân GPIO cho động cơ 1 quay tới
        "backward": DigitalOutputDevice(6),  # Chân GPIO cho động cơ 1 quay lùi
        "enable": PWMOutputDevice(0)          # Chân GPIO để điều khiển tốc độ động cơ 1
    },
    "motor_2": {
        "forward": DigitalOutputDevice(7),   # Chân GPIO cho động cơ 2 quay tới
        "backward": DigitalOutputDevice(8),  # Chân GPIO cho động cơ 2 quay lùi
        "enable": PWMOutputDevice(1)          # Chân GPIO để điều khiển tốc độ động cơ 2
    },
    "motor_3": {
        "forward": DigitalOutputDevice(16),  # Chân GPIO cho động cơ 3 quay tới
        "backward": DigitalOutputDevice(25), # Chân GPIO cho động cơ 3 quay lùi
        "enable": PWMOutputDevice(12)         # Chân GPIO để điều khiển tốc độ động cơ 3
    },
    "motor_4": {
        "forward": DigitalOutputDevice(26),  # Chân GPIO cho động cơ 4 quay tới
        "backward": DigitalOutputDevice(19), # Chân GPIO cho động cơ 4 quay lùi
        "enable": PWMOutputDevice(13)         # Chân GPIO để điều khiển tốc độ động cơ 4
    }
}

# === Servo Configuration ===
# Cấu hình các servo với chân GPIO tương ứng
SERVOS = {
    "servo_1": Servo(14),  # Servo dưới
    "servo_2": Servo(15)   # Servo trên
}

# === Ultrasonic Sensor Configuration ===
# Cấu hình các cảm biến siêu âm với chân GPIO cho echo và trigger
ULTRASONIC_SENSORS = {
    "front": DistanceSensor(echo=11, trigger=9),    # Cảm biến siêu âm phía trước
    "left": DistanceSensor(echo=27, trigger=17),     # Cảm biến siêu âm phía trái
    "right": DistanceSensor(echo=24, trigger=23)     # Cảm biến siêu âm phía phải
}

# === Relay Configuration ===
# Cấu hình relay với chân GPIO tương ứng
RELAY = DigitalOutputDevice(18)  # Relay để kiểm soát các thiết bị khác

# === Water Level Sensor Configuration ===
# Cấu hình cảm biến mức nước với chân GPIO tương ứng
WATER_LEVEL_SENSOR = DigitalOutputDevice(22)  # Cảm biến mức nước
