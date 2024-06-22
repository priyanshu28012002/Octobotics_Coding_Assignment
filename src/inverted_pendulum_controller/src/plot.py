#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from inverted_pendulum_sim.msg import CurrentState
from matplotlib.animation import FuncAnimation

class LivePlotter:
    def __init__(self):
        self.timestamps = []
        self.cart_velocities = []
        self.cart_accelerations = []
        self.pendulum_velocities = []
        self.pendulum_accelerations = []
        self.time_counter = 0

        rospy.init_node('live_plot_node', anonymous=True)
        rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, self.state_callback)

        self.fig = plt.figure(figsize=(12, 10))
        self.ani = FuncAnimation(self.fig, self.update_plots, interval=100)  # Update every 100ms

    def state_callback(self, msg):
        self.time_counter += 0.1

        self.timestamps.append(self.time_counter)
        self.cart_velocities.append(msg.curr_x_dot)
        self.cart_accelerations.append(msg.curr_x_dot_dot)
        self.pendulum_velocities.append(msg.curr_theta_dot)
        self.pendulum_accelerations.append(msg.curr_theta_dot_dot)

    def update_plots(self, frame):
        plt.clf()

        plt.subplot(2, 2, 1)
        plt.plot(self.timestamps, self.cart_velocities)
        plt.title('Cart Velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.grid(True)

        plt.subplot(2, 2, 2)
        plt.plot(self.timestamps, self.cart_accelerations)
        plt.title('Cart Acceleration')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s^2)')
        plt.grid(True)

        plt.subplot(2, 2, 3)
        plt.plot(self.timestamps, self.pendulum_velocities)
        plt.title('Pendulum Angular Velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.grid(True)

        plt.subplot(2, 2, 4)
        plt.plot(self.timestamps, self.pendulum_accelerations)
        plt.title('Pendulum Angular Acceleration')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Acceleration (rad/s^2)')
        plt.grid(True)

        plt.tight_layout()

    def show(self):
        plt.show()

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    plotter = LivePlotter()
    plotter.show()
    plotter.spin()
