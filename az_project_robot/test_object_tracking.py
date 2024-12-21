import os
import cv2
from tensorflow.lite.python.interpreter import Interpreter  # type: ignore
from time import sleep, time
from src.utils.image_utils import (
    load_labels,
    load_model,
    get_model_details,
    detect_objects,
    preprocess_frame,
    calculate_fps,
    draw_detections,
)
from src.hardware.servos import ServoControl
from src.utils.control_utils import set_motors_direction

# Servo setup
servo_1 = ServoControl(channel=1)  # Servo for horizontal movement
servo_2 = ServoControl(channel=0)  # Servo for vertical movement
DEFAULT_ANGLE = 60
MIN_ANGLE = 0
MAX_ANGLE = 120

# Camera configuration
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Detection parameters
MODEL_NAME = '/home/az/Desktop/Project/az_project_robot/models'
GRAPH_NAME = 'detect.tflite'
LABELMAP_NAME = 'labelmap.txt'
min_conf_threshold = 0.7
imW, imH = 640, 480
CENTER_X, CENTER_Y = imW // 2, imH // 2
MIN_BOX_AREA = 5000

# Load labels and model
labels = load_labels(os.path.join(MODEL_NAME, LABELMAP_NAME))
interpreter = load_model(os.path.join(MODEL_NAME, GRAPH_NAME))
input_details, output_details, height, width, floating_model = get_model_details(interpreter)

# State variables
prev_time = time()
servo_angle_1 = DEFAULT_ANGLE
servo_angle_2 = DEFAULT_ANGLE


def adjust_robot_movement(deviation_x, deviation_y, area):
    """Adjust robot movement based on object position and size."""
    if abs(deviation_x) > 30:
        if deviation_x < 0:
            set_motors_direction('move_left', 0.2, 0, 1)
        else:
            set_motors_direction('move_right', 0.2, 0, 1)
    elif area < 30000:  # If the object is too small, move forward
        set_motors_direction('move_forward', 0.3, 0, 1)
    elif area > 80000:  # If the object is too large, move backward
        set_motors_direction('move_backward', 0.3, 0, 1)
    else:
        set_motors_direction('stop', 0, 0, 0)


try:
    while True:
        # Capture frame
        _, frame = cap.read()
        if frame is None:
            print("Cannot read frame!")
            break

        # Preprocess and detect objects
        input_data = preprocess_frame(frame, width, height, floating_model)
        detections = detect_objects(interpreter, input_data, input_details, output_details, min_conf_threshold, imW, imH)

        # Process detections for "Water" and "Charger"
        for target_label in ["Water", "Charger"]:
            for xmin, ymin, xmax, ymax, class_id, score in detections:
                if labels[int(class_id)] == target_label:
                    # Compute object center, deviations, and area
                    obj_center_x = int((xmin + xmax) / 2 * imW)
                    obj_center_y = int((ymin + ymax) / 2 * imH)
                    deviation_x = obj_center_x - CENTER_X
                    deviation_y = obj_center_y - CENTER_Y
                    area = (xmax - xmin) * (ymax - ymin) * imW * imH

                    # Draw bounding box and label
                    cv2.rectangle(frame, (int(xmin * imW), int(ymin * imH)),
                                  (int(xmax * imW), int(ymax * imH)), (0, 255, 0), 2)
                    cv2.putText(frame, f"{target_label}: {score:.2f}",
                                (int(xmin * imW), int(ymin * imH) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Adjust servo for tracking
                    if abs(deviation_x) > 20:
                        if deviation_x < 0:
                            servo_angle_1 = min(MAX_ANGLE, servo_angle_1 + 1)
                        else:
                            servo_angle_1 = max(MIN_ANGLE, servo_angle_1 - 1)
                        servo_1.move_to_angle(servo_angle_1)
                        sleep(0.05)

                    if abs(deviation_y) > 20:
                        if deviation_y < 0:
                            servo_angle_2 = max(MIN_ANGLE, servo_angle_2 - 1)
                        else:
                            servo_angle_2 = min(MAX_ANGLE, servo_angle_2 + 1)
                        servo_2.move_to_angle(servo_angle_2)
                        sleep(0.05)

                    # Adjust robot movement
                    adjust_robot_movement(deviation_x, deviation_y, area)

        # Calculate FPS
        current_time = time()
        fps = calculate_fps(prev_time, current_time, 1)
        prev_time = current_time

        # Display info
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Object Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
