from hardware.time_manager import TimeManager
from modes import Modes
from time import sleep

"""
Tưới cây: Robot thực hiện nhiệm vụ tưới cây theo yêu cầu, với thời gian xác định cho mỗi lần tưới.
Tránh chướng ngại vật: Robot phải liên tục tránh các vật cản trong quá trình di chuyển.
Tìm kiếm trạm sạc: Khi hoàn thành nhiệm vụ, robot sẽ tự động tìm đường về trạm sạc.
Theo dõi thời gian: Robot cần theo dõi thời gian làm việc và thời gian nghỉ ngơi, tự động chuyển đổi giữa các nhiệm vụ.
"""

class Tasks:
    def __init__(self):
        self.modes = Modes()
        self.time_manager = TimeManager(work_time=600, rest_time=300, active_hours=[(8, 17)])  # 10 phút làm việc, 5 phút nghỉ
        self.is_working = False

    def run(self):
        while True:
            if self.time_manager.is_within_active_hours():
                if not self.is_working:
                    print("Starting work cycle...")
                    self.is_working = True
                self.modes.automatic_mode()
            else:
                if self.is_working:
                    print("Ending work cycle, entering rest mode.")
                    self.is_working = False
                self.time_manager.manage_schedule()

    def return_to_station(self):
        print("Returning to charging station...")
        # Logic để quay về trạm sạc
        while not self.modes.charge_station_found:
            # Tìm kiếm đường line màu đỏ để quay về trạm sạc
            if self.modes.color_detection_loop("red"):  # Giả sử có hàm color_detection_loop để phát hiện đường line
                self.modes.follow_line_to_station()  # Logic di chuyển theo đường line
            else:
                self.modes.avoid_obstacles()  # Tránh chướng ngại vật trong quá trình tìm đường

        print("Robot has arrived at the charging station.")
        self.modes.motors.stop_all()  # Dừng tất cả động cơ
        print("Robot is charging...")


