from src.config.gpio_config import RELAY  # Nhập cấu hình từ gpio_config
from time import sleep

class RelayControl:
    def __init__(self, duration = None):
        """
        Khởi tạo relay từ cấu hình GPIO.
        """
        if duration is None:
            self.duration = 10
        self.duration = duration
        try:
            self.relay = RELAY  # Relay được cấu hình trong gpio_config
        except Exception as e:
            print(f"Error initializing relay: {e}")
            self.relay = None

    def toggle_relay(self, state):
        """
        Bật hoặc tắt relay.
        :param state: True để bật, False để tắt relay
        """
        if not self.relay:
            print("Relay not initialized.")
            return

        try:
            if state:
                self.relay.on()
                print("Relay turned ON.")
            else:
                self.relay.off()
                print("Relay turned OFF.")
        except Exception as e:
            print(f"Error toggling relay: {e}")
    def run_relay_for_duration(self):
        """
        Bật relay trong một khoảng thời gian nhất định.
        :param duration: Thời gian bật relay (giây).
        """
        print(f"Relay turned ON for {self.duration} seconds.")
        self.toggle_relay(True)
        sleep(self.duration)
        self.toggle_relay(False)