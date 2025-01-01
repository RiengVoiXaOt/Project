    def return_to_charger_station(self, status_yellow, deviation_x_yellow, deviation_y_yellow, left_distance, right_distance,
                         status_charger, status_red, contours_red, front_distance, front_left_distance, front_right_distance, deviation_x_charger):
        
        if status_yellow and self.current_state != "following red line" and not self.is_tracking_charger:
            self.handle_yellow_line(status_yellow, deviation_x_yellow, deviation_y_yellow, left_distance, right_distance)
            self.current_state = "avoid_line"
        # Khi không có đường màu vàng
        elif not status_yellow:
            self.check_tracking_charger(status_charger)
            # self.check_tracking_red_line(status_red)
            if status_charger:
                self.update_state("Đã phát hiện trạm sạc")
                self.rest_in_charger(front_distance, deviation_x_charger)
                self.current_state = "resting"
            elif status_red and not self.is_tracking_charger:
                self.red_line_following(contours_red)
                self.current_state = "following red line"  
                if contours_red is None or len(contours_red) == 0:
                    self.update_state("Không phát hiện đường đỏ, bắt đầu quay tìm đường")
                    self.set_motors_direction('stop', 0, 0, 0)  # Dừng lại
                    sleep(1)  # Dừng lại trong 1 giây để ổn định
                    while True:
                        # Lấy khung hình và xử lý để phát hiện đường đỏ
                        frame_dict = self.process_frame()  # Xử lý khung hình từ hàng đợi
                        contours_red = frame_dict["contours_red"]  # Lấy contour màu đỏ từ từ điển

                        # Nếu phát hiện đường đỏ, thoát khỏi vòng lặp
                        if contours_red is not None and len(contours_red) > 0:
                            self.update_state("Đã phát hiện lại đường đỏ")
                            break
                        self.set_motors_direction('rotate_left', 0.1, 0, 0)  # Quay trái
                        sleep(0.1)


            else:
                if not self.is_tracking_charger:
                    self.avoid_and_navigate(front_distance, left_distance, right_distance, front_left_distance, front_right_distance)
                    self.current_state = "move"