import numpy as np
from src.hardware.motors import Motors

class Kinematic:
    def __init__(self, x, y, d, t, motors: Motors = None):
        """
        Khởi tạo động học của robot 4 bánh xe đa hướng.
        :param x: Tốc độ trục x toàn cục.
        :param y: Tốc độ trục y toàn cục.
        :param d: Tốc độ góc (rad/s).
        :param t: Chế độ quay (0: không quay, 1: quay).
        :param motors: Đối tượng Motors để điều khiển động cơ.
        """
        if motors is None:
            motors = Motors()  # Gán giá trị mặc định cho motors nếu không có tham số
        
        self.motors = motors
        self.turn = t
        self.x = x
        self.y = y
        self.theta = -np.pi / 2  # Góc khởi đầu (radians)
        self.r = 0.0485  # Bán kính bánh xe (m)
        self.l = 0.1664  # Khoảng cách từ tâm đến bánh xe (m)
        self.v1 = self.v2 = self.v3 = self.v4 = 0
        self.vxg = x
        self.vyg = y
        self.theta_d = d
        self.max_speed_rpm = 60  # Tốc độ tối đa của động cơ (rpm)
        self.max_speed_rad_per_s = self.max_speed_rpm * 2 * np.pi / 60
        # Ma trận cấu hình động học
        self.inv_r = np.array([[0, 1, 0],
                               [-1, 0, 0],
                               [0, 0, 1]])
        self.j2 = np.array([[0.0485, 0, 0, 0],
                            [0, 0.0485, 0, 0],
                            [0, 0, 0.0485, 0],
                            [0, 0, 0, 0.0485]])
        self.j1 = np.array([[0.70710678,  0.70710678, -0.0294669],
                            [-0.70710678,  0.70710678,  0.0294669],
                            [-0.70710678, -0.70710678, -0.0294669],
                            [0.70710678, -0.70710678,  0.0294669]])
        self.v = np.zeros((4, 1))  # Tốc độ của từng bánh xe

    def balancing_velocity(self):
        """
        Cân bằng tốc độ giữa trục x và y nếu cả hai đều không bằng 0.
        """
        if self.vxg != 0 and self.vyg != 0:
            self.vxg /= 2
            self.vyg /= 2

    def backward_kinematics(self):
        """
        Tính toán động học ngược và điều khiển bánh xe dựa trên tốc độ toàn cục.
        """
        # Tính toán tốc độ của từng bánh xe
        b_k = np.linalg.inv(self.j2) @ self.j1 @ np.linalg.inv(self.inv_r) @ np.array([[self.vxg], [self.vyg], [self.theta_d]])
        self.v1, self.v2, self.v3, self.v4 = b_k.flatten() / self.max_speed_rad_per_s

        # Điều khiển động cơ dựa trên hướng quay của bánh xe
        self.motors.set_speed("motor_1", abs(self.v1))
        self.motors.set_direction("motor_1", self.v1 >= 0)

        self.motors.set_speed("motor_2", abs(self.v2))
        self.motors.set_direction("motor_2", self.v2 >= 0)

        self.motors.set_speed("motor_3", abs(self.v3))
        self.motors.set_direction("motor_3", self.v3 >= 0 if self.turn == 1 else self.v3 < 0)

        self.motors.set_speed("motor_4", abs(self.v4))
        self.motors.set_direction("motor_4", self.v4 >= 0 if self.turn == 1 else self.v4 < 0)

    def forward_kinematics(self):
        """
        Tính toán động học thuận để lấy tốc độ toàn cục từ tốc độ bánh xe.
        """
        f_k = self.inv_r @ np.linalg.pinv(self.j1) @ self.j2 @ self.v
        self.vx = f_k[0, 0]
        self.vy = f_k[1, 0]
        self.theta_dot = f_k[2, 0]

    def stop(self):
        """
        Dừng tất cả các bánh xe.
        """
        self.motors.stop_all()
