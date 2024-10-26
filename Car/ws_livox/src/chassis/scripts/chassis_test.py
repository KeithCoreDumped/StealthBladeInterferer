from donkeycar.parts import pins
import sys, tty, termios
from select import select

throttle_pin = pins.pwm_pin_by_id("PCA9685.1:40.1")
throttle_pin.start()

steering_pin = pins.pwm_pin_by_id("PCA9685.1:40.0")
steering_pin.start()

def get_key(settings, timeout=None):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#test_pin = steering_pin
test_pin = throttle_pin

settings = termios.tcgetattr(sys.stdin)
pulse = 380
throttle_pin.duty_cycle(pulse / 4096)
while True:
    k = get_key(settings)
    if k == "q":
        break
    elif k == 'i':
        pulse += 5
    elif k == 'u':
        pulse += 1
    elif k == 'o':
        pulse += 10
    elif k == ',':
        pulse -= 5
    elif k == 'm':
        pulse -= 1
    elif k == '.':
        pulse -= 10
    print("+uio -m,.", pulse)
    test_pin.duty_cycle(pulse / 4096)
    
