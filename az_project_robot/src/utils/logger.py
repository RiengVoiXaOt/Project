import logging
from datetime import datetime

class Logger:
    def __init__(self, log_file="robot.log"):
        """
        Khởi tạo đối tượng Logger.
        :param log_file: Tên file log để ghi nhật ký.
        """
        self.log_file = log_file
        logging.basicConfig(
            filename=self.log_file,
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        self.console = logging.StreamHandler()
        self.console.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        self.console.setFormatter(formatter)
        logging.getLogger().addHandler(self.console)

    def log_info(self, message):
        """
        Ghi log mức độ thông tin.
        :param message: Nội dung thông tin cần ghi log.
        """
        logging.info(message)

    def log_warning(self, message):
        """
        Ghi log mức độ cảnh báo.
        :param message: Nội dung cảnh báo cần ghi log.
        """
        logging.warning(message)

    def log_error(self, message):
        """
        Ghi log mức độ lỗi.
        :param message: Nội dung lỗi cần ghi log.
        """
        logging.error(message)

    def log_critical(self, message):
        """
        Ghi log mức độ nghiêm trọng.
        :param message: Nội dung cần ghi log với mức độ nghiêm trọng.
        """
        logging.critical(message)

    def log_debug(self, message):
        """
        Ghi log mức độ gỡ lỗi (debug).
        :param message: Nội dung gỡ lỗi cần ghi log.
        """
        logging.debug(message)

# Ví dụ sử dụng
if __name__ == "__main__":
    logger = Logger()
    logger.log_info("Robot initialized.")
    logger.log_warning("Battery level low.")
    logger.log_error("Obstacle detection failed.")
    logger.log_critical("System shutdown due to critical error.")
    logger.log_debug("Debugging motor control module.")
