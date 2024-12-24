from time import sleep, time
from src.hardware.relay import RelayControl
from src.hardware.motors import Motors
from src.hardware.ultrasonic import UltrasonicSensors
from src.vision.color_detection import color_detection_loop
from src.vision.object_detection import object_detection_loop
from src.utils.control_utils import getch, direction, set_motors_direction
from src.hardware.servos import ServoControl
from src.utils.image_utils import load_labels, load_model, get_model_details, draw_detections, preprocess_frame, calculate_fps, analyze_detection
from src.vision.video_stream import VideoStream
from src.hardware.battery import BatteryMonitor
from threading import Thread, Event, Lock
from datetime import datetime
from queue import Queue
import numpy as np
import os
import cv2

class RobotController:
    def __init__(self):
        self.stop_event = Event()
        self.manual_mode = False
        self.is_resting = False
        self.mission = True
        self.daily_mission = 10
        self.current_mission_count = 0
        self.firtstart = True

        # Servo-related parameters
        self.MIN_ANGLE = 0
        self.MAX_ANGLE = 120
        self.DEFAULT_ANGLE_BOTTOM = 60
        self.DEFAULT_ANGLE_TOP = 90
        self.target_angle_1 = self.DEFAULT_ANGLE_BOTTOM
        self.target_angle_2 = self.DEFAULT_ANGLE_TOP

        # State tracking
        self.status_charger_history = []
        self.status_water_history = []
        self.servo_angle_history_bottom = []
        self.servo_angle_history_top = []
        self.MAX_HISTORY = 5
        self.reset_threshold = 3

        # Other parameters
        self.OPERATION_START_TIME = datetime.strptime("06:00", "%H:%M").time()
        self.OPERATION_END_TIME = datetime.strptime("20:00", "%H:%M").time()
        self.SAFE_DISTANCE = 5.0  # Example value
        
        # Initialize sensors, motors, and other components
        self.init_components()

    def init_components(self):
        self.ultrasonic_sensors = UltrasonicSensors()
        self.bottom_servo = ServoControl(channel=1)
        self.top_servo = ServoControl(channel=0)
        self.relay_control = RelayControl()
        self.battery = BatteryMonitor()

    def start_video_stream(self):
        self.videostream = VideoStream().start()

    def start_color_detection(self):
        return Thread(target=self.color_detection, daemon=True).start()

    def start_object_detection(self):
        return Thread(target=self.object_detection, daemon=True).start()

    def automatic_mode(self):
        self.start_video_stream()
        color_thread = self.start_color_detection()
        object_thread = self.start_object_detection()
        print("Chế độ tự động đang chạy...")

        try:
            while not self.stop_event.is_set():
                self.daily_reset_check()
                if not self.manual_mode:
                    self.handle_automatic_tasks()
                else:
                    print("Chuyển sang chế độ thủ công. Thoát khỏi tự động.")
                    break

                print(f"Manual Mode: {self.manual_mode}, Stop Event: {self.stop_event.is_set()}")

            color_thread.join()
            object_thread.join()
            sleep(0.1)

        except Exception as e:
            print(f"Error in automatic mode: {e}")

        finally:
            self.stop_event.set()
            if hasattr(self, 'videostream'):
                self.videostream.stop()
            cv2.destroyAllWindows()

    def handle_automatic_tasks(self):
        current_time = time()
        if not self.frame_queue.empty():
            frame_dict = self.process_frame()

            distances = {
                "front": self.ultrasonic_sensors.get_distance("front"),
                "left": self.ultrasonic_sensors.get_distance("left"),
                "right": self.ultrasonic_sensors.get_distance("right"),
                "front_left": self.ultrasonic_sensors.get_distance("front_left"),
                "front_right": self.ultrasonic_sensors.get_distance("front_right"),
            }

            if self.firtstart:
                sleep(3)
                self.firtstart = False

            # Extract frame data
            status_red = frame_dict["status_red"]
            status_black = frame_dict["status_black"]
            deviation_x_black = frame_dict["deviation_x_black"]
            deviation_y_black = frame_dict["deviation_y_black"]
            status_charger = frame_dict["status_charger"]
            deviation_x_charger = frame_dict["deviation_x_charger"]
            deviation_y_charger = frame_dict["deviation_y_charger"]
            deviation_x_water = frame_dict["deviation_x_water"]
            deviation_y_water = frame_dict["deviation_y_water"]
            status_water = frame_dict["status_water"]
            contours_red = frame_dict["contours_red"]

            if self.check_battery_and_time():
                if self.mission:
                    self.execute_watering_task(
                        status_black, deviation_x_black, deviation_y_black,
                        status_water, distances
                    )
                else:
                    self.return_to_charger(
                        status_black, deviation_x_black, deviation_y_black,
                        status_red, status_charger, distances
                    )

            else:
                self.return_to_charger(
                    status_black, deviation_x_black, deviation_y_black,
                    status_red, status_charger, distances
                )

    def execute_watering_task(self, status_black, deviation_x_black, deviation_y_black, status_water, distances):
        self.handle_black_line(status_black, deviation_x_black, deviation_y_black)

        if status_water:
            self.move_to_target(deviation_x_water, deviation_y_water, distances["front"])
        else:
            self.avoid_and_navigate(distances)
            if time > 200:
                self.start_search_thread(self.MIN_ANGLE, 30, 11)

    def return_to_charger(self, status_black, deviation_x_black, deviation_y_black, status_red, status_charger, distances):
        self.handle_black_line(status_black, deviation_x_black, deviation_y_black)

        if not status_black:
            self.red_line_following(contours_red)

            if not status_red and not status_charger:
                self.avoid_and_navigate(distances)

            if status_charger:
                self.rest_in_charger(distances["front"])

    def check_battery_and_time(self):
        now = datetime.now()
        if self.OPERATION_START_TIME <= now.time() <= self.OPERATION_END_TIME:
            return self.battery.read_battery_status()[3] > 25
        return False

    def rest_in_charger(self, front_distance):
        if front_distance > 4.7:
            self.set_motors_direction("go_forward", 0.09, 0.09, 0)
        self.set_motors_direction("stop", 0, 0, 0)
        self.is_resting = True

    def reset_daily_mission(self):
        self.current_mission_count = 0
        self.mission = True
        print("Đã reset nhiệm vụ hàng ngày.")

    def daily_reset_check(self):
        current_time = datetime.now()
        if current_time.hour == 0 and current_time.minute == 0:
            self.reset_daily_mission()

    def move_to_target(self, deviation_x, deviation_y, front_distance):
        # Movement logic for target tracking
        pass

    def avoid_and_navigate(self, distances):
        # Obstacle avoidance logic
        pass

    def red_line_following(self, contours_red):
        # Logic for red line following
        pass

    def handle_black_line(self, status_black, deviation_x_black, deviation_y_black):
        # Logic to handle black line detection
        pass
