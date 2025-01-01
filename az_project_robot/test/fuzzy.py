import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Khai báo các biến đầu vào
distance_front = ctrl.Antecedent(np.arange(0, 101, 1), 'distance_front')  # Cảm biến trước
distance_left = ctrl.Antecedent(np.arange(0, 101, 1), 'distance_left')    # Cảm biến trái
distance_right = ctrl.Antecedent(np.arange(0, 101, 1), 'distance_right')  # Cảm biến phải
distance_front_left = ctrl.Antecedent(np.arange(0, 101, 1), 'distance_front_left')  # Cảm biến trước trái
distance_front_right = ctrl.Antecedent(np.arange(0, 101, 1), 'distance_front_right')  # Cảm biến trước phải

# Khai báo các biến đầu ra
turn_angle = ctrl.Consequent(np.arange(-90, 91, 1), 'turn_angle')  # Góc quay (-90: trái, +90: phải)
speed = ctrl.Consequent(np.arange(0, 101, 1), 'speed')  # Tốc độ (0: dừng, 100: nhanh)

# Hàm thuộc tính mờ cho đầu vào (khoảng cách)
for sensor in [distance_front, distance_left, distance_right, distance_front_left, distance_front_right]:
    sensor['near'] = fuzz.trapmf(sensor.universe, [0, 0, 15, 30])
    sensor['medium'] = fuzz.trapmf(sensor.universe, [20, 30, 60, 70])
    sensor['far'] = fuzz.trapmf(sensor.universe, [60, 80, 100, 100])

# Hàm thuộc tính mờ cho đầu ra (góc quay)
turn_angle['sharp_left'] = fuzz.trimf(turn_angle.universe, [-90, -90, -45])
turn_angle['left'] = fuzz.trimf(turn_angle.universe, [-60, -30, 0])
turn_angle['straight'] = fuzz.trimf(turn_angle.universe, [-10, 0, 10])
turn_angle['right'] = fuzz.trimf(turn_angle.universe, [0, 30, 60])
turn_angle['sharp_right'] = fuzz.trimf(turn_angle.universe, [45, 90, 90])

# Hàm thuộc tính mờ cho đầu ra (tốc độ)
speed['stop'] = fuzz.trimf(speed.universe, [0, 0, 20])
speed['slow'] = fuzz.trimf(speed.universe, [10, 30, 50])
speed['fast'] = fuzz.trimf(speed.universe, [40, 70, 100])

# Các quy tắc điều khiển
rules = [
    # Trường hợp cảm biến trước gần
    ctrl.Rule(distance_front['near'], speed['stop']),
    ctrl.Rule(distance_front['near'] & distance_left['far'], turn_angle['right']),
    ctrl.Rule(distance_front['near'] & distance_right['far'], turn_angle['left']),
    ctrl.Rule(distance_front['near'] & distance_left['medium'] & distance_right['medium'], turn_angle['sharp_left']),
    
    # Trường hợp cảm biến trước trái hoặc trước phải gần
    ctrl.Rule(distance_front_left['near'], turn_angle['right']),
    ctrl.Rule(distance_front_right['near'], turn_angle['left']),
    ctrl.Rule(distance_front_left['medium'] & distance_front_right['medium'], turn_angle['straight']),
    
    # Trường hợp cả trái và phải gần
    ctrl.Rule(distance_left['near'] & distance_right['near'], turn_angle['sharp_right']),
    
    # Trường hợp không có vật cản phía trước
    ctrl.Rule(distance_front['far'] & distance_left['far'] & distance_right['far'], speed['fast']),
    ctrl.Rule(distance_front['far'] & distance_front_left['far'] & distance_front_right['far'], turn_angle['straight']),
]

# Tạo hệ thống điều khiển
turn_control = ctrl.ControlSystem(rules)
robot_control = ctrl.ControlSystemSimulation(turn_control)

# Thử nghiệm với giá trị cảm biến
robot_control.input['distance_front'] = 20  # Giá trị cảm biến trước
robot_control.input['distance_left'] = 50   # Giá trị cảm biến trái
robot_control.input['distance_right'] = 80  # Giá trị cảm biến phải
robot_control.input['distance_front_left'] = 30  # Giá trị cảm biến trước trái
robot_control.input['distance_front_right'] = 15  # Giá trị cảm biến trước phải

# Tính toán đầu ra
robot_control.compute()

# In kết quả
print(f"Góc quay: {robot_control.output['turn_angle']:.2f}°")
print(f"Tốc độ: {robot_control.output['speed']:.2f}")
