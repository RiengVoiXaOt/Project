from ina219 import INA219  # type: ignore
from collections import deque

# Địa chỉ I2C của INA219
INA219_ADDRESS = 0x40  # Địa chỉ mặc định của INA219
SHUNT_OHMS = 0.1
FULL_VOLTAGE = 16.8
MIN_VOLTAGE = 14.8
BATTERY_CAPACITY_AH = 13.6

class BatteryMonitor:
    def __init__(self):
        """
        Khởi tạo cảm biến INA219 và cấu hình các thông số cần thiết.
        """
        try:
            self.ina = INA219(SHUNT_OHMS, address=INA219_ADDRESS, busnum=1)  # Bus 1 cho INA219
            self.ina.configure()
            self.current_history = deque(maxlen=20)  # Lưu trữ dòng điện để tính trung bình
        except Exception as e:
            print(f"Error initializing INA219: {e}")
            self.ina = None

    def read_battery_status(self):
        """
        Đọc trạng thái pin bao gồm điện áp, dòng điện, công suất, phần trăm pin và thời gian còn lại.
        """
        if not self.ina:
            print("INA219 not initialized.")
            return None, None, None, 0, float('inf')

        try:
            voltage = self.ina.voltage()  # Điện áp bus (V)
            current = self.ina.current()  # Dòng điện (mA)
            power = self.ina.power()      # Công suất (mW)
            self.current_history.append(current)

            # Tính dòng điện trung bình
            average_current = sum(self.current_history) / len(self.current_history)

            # Tính phần trăm pin
            battery_percentage = max(0, (voltage - MIN_VOLTAGE) / (FULL_VOLTAGE - MIN_VOLTAGE) * 100)

            # Tính dung lượng còn lại (Ah)
            remaining_capacity_ah = BATTERY_CAPACITY_AH * (battery_percentage / 100)

            # Tính thời gian còn lại (giờ)
            remaining_time_hours = (remaining_capacity_ah / (average_current / 1000)) if average_current > 0 else float('inf')

            return voltage, current, power, battery_percentage, remaining_time_hours

        except Exception as e:
            print(f"Error reading battery status: {e}")
            return None, None, None, 0, float('inf')

    def display_battery_status(self):
        """
        Hiển thị trạng thái pin với thông tin chi tiết.
        """
        voltage, current, power, battery_percentage, remaining_time_hours = self.read_battery_status()
        if voltage is not None:
            print("Battery Status:")
            print(f"  Bus Voltage: {voltage:.2f} V")
            print(f"  Current: {current:.2f} mA")
            print(f"  Power: {power:.2f} mW")
            print(f"  Battery Percentage: {battery_percentage:.2f}%")
            print(f"  Estimated Remaining Time: {remaining_time_hours:.2f} hours")
            print("-" * 40)
        else:
            print("Failed to read battery status.")