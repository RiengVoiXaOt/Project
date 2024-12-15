from gpiozero import DigitalOutputDevice # type: ignore
from config.gpio_config import RELAY  # Nhập cấu hình từ gpio_config

class RelayControl:
    def __init__(self):
        """
        Khởi tạo relay từ cấu hình GPIO.
        """
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