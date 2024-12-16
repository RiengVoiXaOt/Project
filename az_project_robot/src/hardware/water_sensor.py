from gpiozero import DigitalInputDevice # type: ignore
from src.config.gpio_config import WATER_LEVEL_SENSOR  # Nhập cấu hình từ gpio_config

class WaterLevelSensor:
    def __init__(self):
        """
        Khởi tạo cảm biến mực nước dưới dạng tín hiệu số từ cấu hình GPIO.
        """
        try:
            self.sensor = WATER_LEVEL_SENSOR  # Lấy cảm biến từ cấu hình
        except Exception as e:
            print(f"Error initializing water level sensor: {e}")
            self.sensor = None

    def is_water_present(self):
        """
        Kiểm tra xem nước có được phát hiện hay không.
        :return: True nếu có nước, False nếu không có nước.
        """
        if not self.sensor:
            print("Water level sensor not initialized.")
            return False
        try:
            return self.sensor.value == 1
        except Exception as e:
            print(f"Error reading water level sensor: {e}")
            return False
