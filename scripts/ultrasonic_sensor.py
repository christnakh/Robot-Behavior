import rospy
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

TRIGGER = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

class UltrasonicSensor:
    def __init__(self):
        self.range_pub = rospy.Publisher('ultrasonic_range', Range, queue_size=10)
        rospy.init_node('ultrasonic_sensor', anonymous=True)
        self.rate = rospy.Rate(10)

    def measure_distance(self):
        GPIO.output(TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(TRIGGER, False)

        start_time = time.time()
        stop_time = time.time()

        while GPIO.input(ECHO) == 0:
            start_time = time.time()

        while GPIO.input(ECHO) == 1:
            stop_time = time.time()

        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2
        return distance

    def run(self):
        while not rospy.is_shutdown():
            distance = self.measure_distance()
            range_msg = Range()
            range_msg.range = distance
            self.range_pub.publish(range_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        sensor = UltrasonicSensor()
        sensor.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
