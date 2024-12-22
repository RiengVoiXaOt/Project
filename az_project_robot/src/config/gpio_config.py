from gpiozero import DigitalOutputDevice, PWMOutputDevice, DistanceSensor  # type: ignore

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
        "enable": PWMOutputDevice(18)         # Chân GPIO để điều khiển tốc độ động cơ 3
    
    },
    "motor_3": {
        "forward": DigitalOutputDevice(0),   # Chân GPIO cho động cơ 3 quay tới
        "backward": DigitalOutputDevice(1),  # Chân GPIO cho động cơ 3 quay lùi
        "enable": PWMOutputDevice(12)          # Chân GPIO để điều khiển tốc độ động cơ 2
    },
    "motor_4": {
        "forward": DigitalOutputDevice(20),  # Chân GPIO cho động cơ 4 quay tới
        "backward": DigitalOutputDevice(21), # Chân GPIO cho động cơ 4 quay lùi
        "enable": PWMOutputDevice(19)         # Chân GPIO để điều khiển tốc độ động cơ 4
    }
}

# === Ultrasonic Sensor Configuration ===
# Cấu hình các cảm biến siêu âm với chân GPIO cho echo và trigger
F_SENSOR = DistanceSensor(echo=9, trigger=10)    # Cảm biến phía trước
F_R_SENSOR = DistanceSensor(echo=15, trigger=14)
F_L_SENSOR = DistanceSensor(echo=24, trigger=23)
L_SENSOR = DistanceSensor(echo=25, trigger=4)    # Cảm biến bên trái
R_SENSOR = DistanceSensor(echo=22, trigger=8)   # Cảm biến bên phải

# === Relay Configuration ===
# Cấu hình relay với chân GPIO tương ứng
RELAY = DigitalOutputDevice(5)  # Relay để kiểm soát các thiết bị khác

# === Water Level Sensor Configuration ===
# Mức nước đo bằng GPIO 11
WATER_LEVEL_SENSOR = DigitalOutputDevice(11)
