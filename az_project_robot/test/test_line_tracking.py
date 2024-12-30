def find_red_line():
    while True:
        move_zigzag()
        if detect_red_line():
            return True
    return False
def move_zigzag():
    move_forward()
    adjust_left()
    move_forward()
    adjust_right()
    move_forward()
    adjust_left()
    move_forward()
    adjust_right()
def follow_red_line():
    while True:
        position = detect_line_position()
        if position == "CENTER":
            move_forward()
        elif position == "LEFT":
            adjust_left()
        elif position == "RIGHT":
            adjust_right()
        else:  # Line lost
            stop()
            return "LINE_LOST"

        if detect_station_label():  # Check if reached station
            stop()
            return "STATION_FOUND"

def search_and_follow_line():
    while True:
        if find_red_line():
            result = follow_red_line()
            if result == "STATION_FOUND":
                break
            elif result == "LINE_LOST":  # Line lost, retry zigzag search
                rotate_180()
                continue
