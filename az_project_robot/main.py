from time import sleep
from robot.modes import Modes
from robot.tasks import Tasks
from utils.logger import Logger

def main():
    # Khởi tạo logger
    logger = Logger()
    logger.log_info("Robot system starting...")

    # Khởi tạo các chế độ và nhiệm vụ
    modes = Modes()
    tasks = Tasks()

    try:
        while True:
            # Kiểm tra chế độ hoạt động
            if modes.manual_mode:
                logger.log_info("Manual mode activated.")
                modes.manual_control()  # Điều khiển robot bằng tay
            else:
                logger.log_info("Automatic mode activated.")
                tasks.run()  # Thực hiện các nhiệm vụ tự động

            # Hiển thị menu điều khiển
            print("\nMenu:")
            print("1. Switch to Manual Mode")
            print("2. Switch to Automatic Mode")
            print("3. Quit")
            user_input = input("Choose an option (1/2/3): ").strip()

            if user_input == '1':
                modes.switch_to_manual()  # Chuyển sang chế độ thủ công
                logger.log_info("Switched to manual mode.")

            elif user_input == '2':
                modes.switch_to_automatic()  # Chuyển sang chế độ tự động
                logger.log_info("Switched to automatic mode.")

            elif user_input == '3':
                logger.log_info("Shutting down robot system...")
                break  # Thoát khỏi vòng lặp và dừng robot

            else:
                print("Invalid input! Please enter 1, 2, or 3.")

            # Kiểm tra thời gian nghỉ và hoạt động
            sleep(1)  # Thời gian chờ giữa các vòng lặp

    except KeyboardInterrupt:
        logger.log_info("Shutting down robot system...")
    finally:
        modes.motors.stop_all()  # Dừng tất cả động cơ
        logger.log_info("Robot system stopped.")

if __name__ == "__main__":
    main()
