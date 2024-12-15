from hardware.kinematic import Kinematic
from hardware.motors import Motors
from utils.control_utils import getch, direction, set_motors_direction
from robot.modes import Modes
from robot.tasks import Tasks
from utils.logger import Logger
from time import sleep

def display_menu():
    print("\nMenu:")
    print("1. Switch to Manual Mode")
    print("2. Switch to Automatic Mode")
    print("3. Quit")
    
def main():
    # Khởi tạo logger
    logger = Logger()
    logger.log_info("Robot system starting...")

    # Khởi tạo các chế độ và nhiệm vụ
    motors = Motors()
    robot = Kinematic(0, 0, 0, 0, motors)
    modes = Modes(n=5, vx=0.04309596457, vy=0.04309596457, theta=0)
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
            display_menu()
            user_input = input("Choose an option (1/2/3): ").strip()

            if user_input == '1':
                modes.switch_mode()  # Chuyển sang chế độ thủ công
                logger.log_info("Switched to manual mode.")

            elif user_input == '2':
                modes.switch_mode()  # Chuyển sang chế độ tự động
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
        motors.stop_all()  # Dừng tất cả động cơ
        logger.log_info("Robot system stopped.")

if __name__ == "__main__":
    main()
