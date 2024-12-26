    #Tìm đường về trạm sạc
def return_charger(self, status_yellow, deviation_x_yellow, deviation_y_yellow, left_distance, right_distance,
                    status_water, deviation_x_water, deviation_y_water, front_distance,)
    if not self.is_resting:
        self.handle_yellow_line(status_yellow, deviation_x_yellow, deviation_y_yellow)
        if not status_yellow:
            self.red_line_following(contours_red)
            if not status_red and not status_charger:
                self.avoid_and_navigate(front_distance, left_distance, right_distance,front_left_distance,front_right_distance)
            if status_charger:
                set_motors_direction("stop", 0.1, 0.1, 0)
                sleep(0.05)
                if front_distance < 25 and status_charger:
                    self.rest_in_charger(front_distance)
    else:
        self.rest_in_charger(front_distance)
