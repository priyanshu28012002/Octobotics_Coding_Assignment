#!/usr/bin/env python3

import rospy
import numpy as np
from inverted_pendulum_sim.msg import CurrentState, ControlForce

class PIDController:
    def __init__(self, Kp, Ki, Kd, target_theta=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target_theta = target_theta
        self.state = np.zeros(4)
        self.prev_error = 0.0
        self.integral = 0.0

        self.pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
        rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, self.state_callback)
        self.rate = rospy.Rate(100)  # Increase the rate to 100 Hz for faster control

    def state_callback(self, msg):
        self.state = np.array([msg.curr_theta, msg.curr_theta_dot, msg.curr_x, msg.curr_x_dot])

    def compute_control(self):
        theta_error = self.target_theta - self.state[0]  # Error in pendulum angle

        P = self.Kp * theta_error
        self.integral += theta_error
        I = self.Ki * self.integral
        D = self.Kd * (theta_error - self.prev_error)
        self.prev_error = theta_error

        u = P + I + D

        force_msg = ControlForce()
        force_msg.force = u
        self.pub.publish(force_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.compute_control()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pid_controller_node', anonymous=True)

    # PID gains (tuned for faster response)
    Kp = 100.0
    Ki = 0.05
    Kd = 50.0

    pid_controller = PIDController(Kp, Ki, Kd)
    try:
        pid_controller.run()
    except rospy.ROSInterruptException:
        pass
