import cv2
from src.utils.control_utils import getch, direction, set_motors_direction
from src.hardware.motors import Motors
from time import sleep, time

def red_line_following(contours):
        if contours:
            #  update_state("Đang bám theo line đỏ")
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                if cx < 150:  
                    set_motors_direction('rotate_left',0.1 ,0, 0)
                elif cx > 400:  
                      set_motors_direction('rotate_right',0.1,0, 0)
                else:  
                      set_motors_direction('go_forward', 0.1, 0, 0)

