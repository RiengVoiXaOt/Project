import cv2
import numpy as np

# Initialize the webcam
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

if not camera.isOpened():
    print("Error: Could not open the webcam.")
    exit()

while True:
    # Capture a frame from the webcam
    ret, frame = camera.read()
    if not ret:
        print("Error: Could not read a frame from the webcam.")
        break

    # Convert the frame to HSV for color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the red color range (adjust as necessary for lighting conditions)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # Create a mask for red color
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = red_mask1 | red_mask2

    # Clean up the mask with morphological operations
    #kernel = np.ones((3, 3), np.uint8)
    #red_mask = cv2.erode(red_mask, kernel, iterations=5)
    #red_mask = cv2.dilate(red_mask, kernel, iterations=9)

    # Find contours of the red line
    contours, _ = cv2.findContours(red_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Get the largest contour (assumed to be the red line)
        largest_contour = max(contours, key=cv2.contourArea)
        blackbox = cv2.minAreaRect(largest_contour)
        (x_min, y_min), (w_min, h_min), ang = blackbox

        # Calculate the angle and error
        if ang < -45:
            ang = 90 + ang
        if w_min < h_min and ang > 0:
            ang = (90 - ang) * -1
        if w_min > h_min and ang < 0:
            ang = 90 + ang

        setpoint = 320  # Horizontal center of the frame
        error = int(x_min - setpoint)
        ang = int(ang)

        # Draw the bounding box and display the angle and error
        box = cv2.boxPoints(blackbox)
        box = np.intp(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)
        cv2.putText(frame, f"Angle: {ang}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, f"Error: {error}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.line(frame, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

    # Display the frame
    cv2.imshow("Red Line Tracking", frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
camera.release()
cv2.destroyAllWindows()
