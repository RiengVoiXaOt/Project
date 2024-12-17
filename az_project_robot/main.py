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
    logger = Logger()
    logger.log_info("Robot system starting...")
    motors = Motors()
    modes = Modes()
    tasks = Tasks()

    try:
        while True:
            # Lựa chọn từ menu
            display_menu()
            user_input = input("Choose an option (1/2/3): ").strip()

            if user_input == '1':  # Chế độ thủ công
                if modes.current_mode != 'manual':
                    modes.switch_mode('manual')
                    logger.log_info("Switched to manual mode.")
                    modes.manual_control()
                else:
                    print("Robot is already in manual mode.")

            elif user_input == '2':  # Chế độ tự động
                if modes.current_mode != 'automatic':
                    modes.switch_mode('automatic')
                    logger.log_info("Switched to automatic mode.")
                    modes.automatic_mode()
                else:
                    print("Robot is already in automatic mode.")

            elif user_input == '3':  # Thoát chương trình
                logger.log_info("Shutting down robot system...")
                break

            else:
                print("Invalid input! Please enter 1, 2, or 3.")

    except KeyboardInterrupt:
        logger.log_info("Shutting down robot system... (KeyboardInterrupt)")
    finally:
        motors.stop_all()
        logger.log_info("Robot system stopped.")

if __name__ == "__main__":
    main()
