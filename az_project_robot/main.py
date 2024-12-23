from src.hardware.motors import Motors
from src.robot.modes import Modes
from src.utils.logger import Logger
from time import sleep

def display_menu():
    print("\nMenu:")
    print("1. Switch to Manual Mode")
    print("2. Switch to Automatic Mode")
    print("3. Quit")

def main():
    # Khởi tạo logger và các đối tượng
    logger = Logger()
    logger.log_info("Robot system starting...")
    motors = Motors()
    modes = Modes(n=3, theta=0)
    
    current_mode = None  # Biến theo dõi chế độ hiện tại

    try:
        while True:
            # Hiển thị menu cho người dùng chọn chế độ
            display_menu()
            user_input = input("Choose an option (1/2/3): ").strip()

            if user_input == '1':
                if current_mode != 'manual':
                    modes.switch_mode('manual')  # Chuyển sang chế độ thủ công
                    logger.log_info("Switched to manual mode.")
                    modes.manual_control()  # Điều khiển thủ công
                    current_mode = 'manual'
                else:
                    print("Đang ở chế độ thủ công.")

            elif user_input == '2':
                if current_mode != 'automatic':
                    modes.switch_mode('automatic')  # Chuyển sang chế độ tự động
                    logger.log_info("Switched to automatic mode.")
                    modes.automatic_mode()  # Chạy chế độ tự động
                    current_mode = 'automatic'
                else:
                    print("Đang ở chế độ tự động.")

            elif user_input == '3':
                logger.log_info("Shutting down robot system...")
                break  # Thoát khỏi vòng lặp và dừng robot

            else:
                print("Invalid input! Please enter 1, 2, or 3.")

            # Thời gian chờ giữa các vòng lặp
            sleep(1)

    except KeyboardInterrupt:
        logger.log_info("Shutting down robot system...")
    except Exception as e:
        logger.log_info(f"An error occurred: {e}")
    finally:
        motors.stop_all()  # Dừng tất cả động cơ
        logger.log_info("Robot system stopped.")

if __name__ == "__main__":
    main()
