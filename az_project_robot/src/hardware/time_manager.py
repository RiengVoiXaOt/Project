import time
from relay import RelayControl  # Import RelayControl để điều khiển relay
from datetime import datetime


class TimeManager:
    def __init__(self, work_time, rest_time):
        """
        Quản lý thời gian làm việc và nghỉ ngơi của hệ thống.
        :param work_time: Thời gian làm việc (giây).
        :param rest_time: Thời gian nghỉ ngơi (giây).
        """
        self.work_time = work_time
        self.rest_time = rest_time
        # Đặt khoảng thời gian hoạt động từ 6h-7h và 14h-15h
        self.active_hours = [(6, 7), (14, 15)]
        self.start_time = None
        self.relay_control = RelayControl()  # Tạo đối tượng RelayControl
        self.relay_start_time = None

    def is_active_time(self, current_hour):
        """
        Kiểm tra xem thời gian hiện tại có nằm trong khoảng thời gian hoạt động không.
        :param current_hour: Giờ hiện tại (0-23).
        :return: True nếu nằm trong khoảng thời gian hoạt động, False nếu không.
        """
        for start, end in self.active_hours:
            if start <= current_hour < end:
                return True
        return False

    def manage_time(self):
        """
        Quản lý thời gian làm việc và nghỉ ngơi dựa trên thời gian hoạt động.
        """
        current_hour = datetime.now().hour  # Lấy giờ hiện tại
        if self.is_active_time(current_hour):
            print("Hệ thống đang hoạt động.")
            # Bật relay hoặc thực hiện các tác vụ khác trong thời gian làm việc
            self.relay_control.toggle_relay(True)
            time.sleep(self.work_time)  # Thực hiện công việc trong thời gian làm việc
            self.relay_control.toggle_relay(False)
            print("Hệ thống đã nghỉ ngơi.")
            time.sleep(self.rest_time)  # Nghỉ ngơi trong thời gian nghỉ
        else:
            print("Hệ thống không hoạt động vào thời gian này.")


    def start_task(self):
        """
        Ghi nhận thời điểm bắt đầu nhiệm vụ và bật relay.
        """
        self.start_time = time.time()
        print("Task started.")
        self.relay_control.toggle_relay(True)  # Bật relay

    def check_task_time(self):
        """
        Kiểm tra nếu thời gian làm việc đã kết thúc và tắt relay sau 10 giây.
        :return: True nếu thời gian làm việc đã hoàn thành, False nếu chưa.
        """
        elapsed_time = time.time() - self.start_time

        # Nếu đã hết thời gian làm việc
        if elapsed_time > self.work_time:
            print("Work time completed.")

            # Nếu relay đang bật, tắt nó sau 10 giây
            if elapsed_time < self.work_time + 10:
                self.relay_control.toggle_relay(False)
                print("Relay turned off after 10 seconds.")

            return True
        return False

    def calculate_rest_time(self):
        """
        Tính toán thời gian nghỉ ngơi còn lại.
        :return: Thời gian nghỉ ngơi còn lại (giây).
        """
        elapsed = time.time() - self.start_time
        remaining_rest = max(0, self.rest_time - elapsed)
        print(f"Remaining rest time: {remaining_rest} seconds.")
        return remaining_rest

    def run_relay_for_duration(self, duration):
        """
        Bật relay trong một khoảng thời gian nhất định.
        :param duration: Thời gian bật relay (giây).
        """
        print(f"Relay turned on for {duration} seconds.")
        self.relay_control.toggle_relay(True)
        time.sleep(duration)
        self.relay_control.toggle_relay(False)
        print("Relay turned off.")

    def is_within_active_hours(self):
        """
        Kiểm tra xem thời gian hiện tại có nằm trong khoảng giờ hoạt động không.
        :return: True nếu trong giờ hoạt động, False nếu không.
        """
        current_hour = time.localtime().tm_hour
        for start_hour, end_hour in self.active_hours:
            if start_hour <= current_hour < end_hour:
                return True
        return False

    def manage_schedule(self):
        """
        Quản lý lịch trình hoạt động và nghỉ ngơi của robot dựa trên giờ thực tế.
        """
        if self.is_within_active_hours():
            print("Robot is within active hours. Starting task...")
            self.start_task()
        else:
            print("Robot is outside active hours. Entering rest mode.")
            remaining_time = self.calculate_rest_time()
            time.sleep(remaining_time)