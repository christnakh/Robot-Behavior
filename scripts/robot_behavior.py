import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time
import math

# Pin definitions for Motors
MOTOR_1_PIN_1 = 5
MOTOR_1_PIN_2 = 6
MOTOR_2_PIN_1 = 13
MOTOR_2_PIN_2 = 19
ENA = 12
ENB = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_1_PIN_1, GPIO.OUT)
GPIO.setup(MOTOR_1_PIN_2, GPIO.OUT)
GPIO.setup(MOTOR_2_PIN_1, GPIO.OUT)
GPIO.setup(MOTOR_2_PIN_2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

class RobotBehavior:
    def __init__(self):
        rospy.init_node('robot_behavior')

        self.human_detected = False
        self.obstacle_distance = None
        self.initial_pose = None
        self.current_pose = None
        self.waypoints = [
            (0.0, 0.0),
            (1.0, 1.0),
            (2.0, 0.0),
            (3.0, -1.0),
            (0.0, 0.0)
        ]
        self.current_waypoint_index = 0
        self.pwm_a = GPIO.PWM(ENA, 100)
        self.pwm_b = GPIO.PWM(ENB, 100)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        self.range_sub = rospy.Subscriber('ultrasonic_range', Range, self.range_callback)
        self.human_sub = rospy.Subscriber('human_detected', Bool, self.human_callback)
        self.pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.pose_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)
        self.avoiding_obstacle = False

    def range_callback(self, msg):
        self.obstacle_distance = msg.range

    def human_callback(self, msg):
        self.human_detected = msg.data
        if self.human_detected:
            self.flee()

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        if self.initial_pose is None:
            self.initial_pose = msg.pose

    def avoid_obstacle(self):
        if self.obstacle_distance is not None and self.obstacle_distance < 30:
            if not self.avoiding_obstacle:
                self.avoiding_obstacle = True

                if self.obstacle_distance < 10:
                    self.move('backward')
                    rospy.sleep(1)
                    self.move('stop')
                else:
                    obstacle_on_left = False  # Placeholder, replace with actual sensor data or logic
                    if obstacle_on_left:
                        self.move('left')
                    else:
                        self.move('right')

                rospy.sleep(1)
                self.avoiding_obstacle = False

    def flee(self):
        twist = Twist()
        twist.linear.x = -0.5
        self.cmd_pub.publish(twist)

    def move(self, direction):
        if direction == 'forward':
            GPIO.output(MOTOR_1_PIN_1, GPIO.HIGH)
            GPIO.output(MOTOR_1_PIN_2, GPIO.LOW)
            GPIO.output(MOTOR_2_PIN_1, GPIO.HIGH)
            GPIO.output(MOTOR_2_PIN_2, GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(50)
            self.pwm_b.ChangeDutyCycle(50)
        elif direction == 'backward':
            GPIO.output(MOTOR_1_PIN_1, GPIO.LOW)
            GPIO.output(MOTOR_1_PIN_2, GPIO.HIGH)
            GPIO.output(MOTOR_2_PIN_1, GPIO.LOW)
            GPIO.output(MOTOR_2_PIN_2, GPIO.HIGH)
            self.pwm_a.ChangeDutyCycle(50)
            self.pwm_b.ChangeDutyCycle(50)
        elif direction == 'right':
            GPIO.output(MOTOR_1_PIN_1, GPIO.HIGH)
            GPIO.output(MOTOR_1_PIN_2, GPIO.LOW)
            GPIO.output(MOTOR_2_PIN_1, GPIO.LOW)
            GPIO.output(MOTOR_2_PIN_2, GPIO.HIGH)
            self.pwm_a.ChangeDutyCycle(50)
            self.pwm_b.ChangeDutyCycle(50)
        elif direction == 'left':
            GPIO.output(MOTOR_1_PIN_1, GPIO.LOW)
            GPIO.output(MOTOR_1_PIN_2, GPIO.HIGH)
            GPIO.output(MOTOR_2_PIN_1, GPIO.HIGH)
            GPIO.output(MOTOR_2_PIN_2, GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(50)
            self.pwm_b.ChangeDutyCycle(50)
        elif direction == 'stop':
            GPIO.output(MOTOR_1_PIN_1, GPIO.LOW)
            GPIO.output(MOTOR_1_PIN_2, GPIO.LOW)
            GPIO.output(MOTOR_2_PIN_1, GPIO.LOW)
            GPIO.output(MOTOR_2_PIN_2, GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)

    def return_to_initial_position(self):
        if self.current_pose is not None and self.initial_pose is not None:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            initial_x = self.initial_pose.position.x
            initial_y = self.initial_pose.position.y

            distance_to_initial = math.sqrt((current_x - initial_x) ** 2 + (current_y - initial_y) ** 2)
            if distance_to_initial > 0.1:
                self.move_towards(initial_x, initial_y)
            else:
                rospy.loginfo("Robot has returned to initial position.")

    def move_towards(self, target_x, target_y):
        if self.current_pose is not None:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y

            delta_x = target_x - current_x
            delta_y = target_y - current_y

            desired_heading = math.atan2(delta_y, delta_x)
            current_heading = 0  # Replace with actual current heading

            angle_difference = desired_heading - current_heading

            if abs(angle_difference) > 0.1:
                if angle_difference > 0:
                    self.move('right')
                else:
                    self.move('left')
            else:
                self.move('forward')

    def plan_and_follow_path(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        while not rospy.is_shutdown() and self.current_waypoint_index < len(self.waypoints):
            waypoint_x, waypoint_y = self.waypoints[self.current_waypoint_index]
            self.move_towards(waypoint_x, waypoint_y)

            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            distance_to_waypoint = math.sqrt((current_x - waypoint_x) ** 2 + (current_y - waypoint_y) ** 2)

            if distance_to_waypoint < 0.2:
                self.current_waypoint_index += 1
                rospy.loginfo(f"Reached waypoint {self.current_waypoint_index}")

                if self.current_waypoint_index >= len(self.waypoints):
                    rospy.loginfo("Reached final waypoint.")
                    break

            rospy.sleep(0.1)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.avoid_obstacle()
            self.return_to_initial_position()
            rate.sleep()

if __name__ == '__main__':
    try:
        behavior = RobotBehavior()
        behavior.plan_and_follow_path()
        behavior.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
