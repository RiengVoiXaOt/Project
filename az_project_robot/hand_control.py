from gpiozero import DigitalOutputDevice, PWMOutputDevice

# Motor 1
in1 = DigitalOutputDevice(5)
in2 = DigitalOutputDevice(6)
en1 = PWMOutputDevice(0)

# Motor 2
in3 = DigitalOutputDevice(7)
in4 = DigitalOutputDevice(8)
en2 = PWMOutputDevice(1)

# Motor 3
in5 = DigitalOutputDevice(16)
in6 = DigitalOutputDevice(25)
en3 = PWMOutputDevice(12)

# Motor 4
in7 = DigitalOutputDevice(19)
in8 = DigitalOutputDevice(26)
en4 = PWMOutputDevice(13)

# Hàm điều khiển robot
def stop_robot():
    in1.off(); in2.off()
    in3.off(); in4.off()
    in5.off(); in6.off()
    in7.off(); in8.off()
    en1.value = 0; en2.value = 0; en3.value = 0; en4.value = 0

def move_robot(direction):
    stop_robot()  # Đảm bảo robot dừng trước khi di chuyển
    speed = 0.3  # Tốc độ PWM cho động cơ

    if direction == 'forward':  # Tiến
        in1.on(); in2.off(); en1.value = speed
        in3.on(); in4.off(); en2.value = speed
        in5.on(); in6.off(); en3.value = speed
        in7.on(); in8.off(); en4.value = speed
    elif direction == 'backward':  # Lùi
        in1.off(); in2.on(); en1.value = speed
        in3.off(); in4.on(); en2.value = speed
        in5.off(); in6.on(); en3.value = speed
        in7.off(); in8.on(); en4.value = speed
    elif direction == 'left':  # Sang trái
        in1.off(); in2.on(); en1.value = speed
        in3.on(); in4.off(); en2.value = speed
        in5.on(); in6.off(); en3.value = speed
        in7.off(); in8.on(); en4.value = speed
    elif direction == 'right':  # Sang phải
        in1.on(); in2.off(); en1.value = speed
        in3.off(); in4.on(); en2.value = speed
        in5.off(); in6.on(); en3.value = speed
        in7.on(); in8.off(); en4.value = speed
    elif direction == 'stop':  # Dừng
        stop_robot()
    elif direction == 'turn_left':  
        in1.off(), in2.on()
        in3.on(), in4.off()
        in5.off(), in6.on()
        in7.on(), in8.off() 
        en1.value = speed
        en2.value = speed
        en3.value = speed
        en4.value = speed
    elif direction == 'turn_right': 
        in1.on(), in2.off()
        in3.off(), in4.on()
        in5.on(), in6.off()
        in7.off(), in8.on()
        en1.value = speed
        en2.value = speed
        en3.value = speed
        en4.value = speed
    elif direction == 'diagonal_right': 
        in1.on(), in2.on()
        in3.on(), in4.off()
        in5.on(), in6.off()
        in7.off(), in8.off()
        en1.value = speed
        en2.value = speed
        en3.value = speed
        en4.value = speed
    elif direction == 'diagonal_left':  
        in1.on(), in2.on()
        in3.on(), in4.off()
        in5.on(), in6.on()
        in7.on(), in8.off()
        en1.value = speed
        en2.value = speed
        en3.value = speed
        en4.value = speed
    elif direction == 'diagonal_right_rev':  
        in1.on(), in2.on()
        in3.off(), in4.on()
        in5.off(), in6.on()
        in7.on(), in8.on()
        en1.value = speed
        en2.value = speed
        en3.value = speed
        en4.value = speed
    elif direction == 'diagonal_left_rev':  
        in1.off(), in2.on()
        in3.on(), in4.on()
        in5.on(), in6.on()
        in7.off(), in8.on()
        en1.value = speed
        en2.value = speed
        en3.value = speed
        en4.value = speed
