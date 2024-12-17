from src.hardware.motors import Motors
from src.robot.modes import Modes
from src.robot.tasks import Tasks
from src.utils.logger import Logger
from time import sleep

def display_menu():
    """Hiển thị menu cho người dùng chọn chế độ hoạt động."""
    print("\nMenu:")
    print("1. Switch to Manual Mode")
    print("2. Switch to Automatic Mode")
    print("3. Quit")

def main():
    # Khởi tạo logger và các đối tượng
    logger = Logger()  # Khởi tạo logger để ghi lại thông tin
    logger.log_info("Robot system starting...")  # Ghi lại thông tin khởi động
    motors = Motors()  # Khởi tạo đối tượng điều khiển động cơ
    modes = Modes()  # Khởi tạo đối tượng Modes để quản lý chế độ hoạt động
    tasks = Tasks()  # Khởi tạo đối tượng Tasks nếu cần cho các tác vụ khác

    try:
        while True:
            # Hiển thị menu cho người dùng chọn chế độ
            display_menu()
            user_input = input("Choose an option (1/2/3): ").strip()  # Nhận đầu vào từ người dùng

            if user_input == '1':
                modes.switch_mode()  # Chuyển sang chế độ thủ công
                logger.log_info("Switched to manual mode.")
                modes.manual_control()  # Cho phép điều khiển robot trong chế độ thủ công
            elif user_input == '2':
                modes.switch_mode()  # Chuyển sang chế độ tự động
                logger.log_info("Switched to automatic mode.")
                modes.automatic_mode()  # Bắt đầu chế độ tự động
            elif user_input == '3':
                logger.log_info("Shutting down robot system...")  # Ghi lại thông tin tắt hệ thống
                break  # Thoát khỏi vòng lặp và dừng robot
            else:
                print("Invalid input! Please enter 1, 2, or 3.")  # Thông báo lỗi nếu đầu vào không hợp lệ

            # Thời gian chờ giữa các vòng lặp
            sleep(1)  # Giảm tải cho CPU và cải thiện trải nghiệm người dùng

    except KeyboardInterrupt:
        logger.log_info("Shutting down robot system...")  # Ghi lại thông tin nếu người dùng dừng bằng Ctrl+C
    finally:
        motors.stop_all()  # Dừng tất cả động cơ khi thoát
        logger.log_info("Robot system stopped.")  # Ghi lại thông tin dừng hệ thống

if __name__ == "__main__":
    main()  # Gọi hàm main khi chạy chương trình
