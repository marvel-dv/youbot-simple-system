#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from tkinter import *

class YoubotTeleop:
    def __init__(self):
        rospy.init_node('youbot_teleop_gui')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

        self.root = Tk()
        self.root.title("YouBot Teleop")

        # Sliders
        Label(self.root, text="Linear X").pack()
        self.slider_x = Scale(self.root, from_=-0.5, to=0.5, resolution=0.01, orient=HORIZONTAL)
        self.slider_x.pack()

        Label(self.root, text="Linear Y").pack()
        self.slider_y = Scale(self.root, from_=-0.5, to=0.5, resolution=0.01, orient=HORIZONTAL)
        self.slider_y.pack()

        Label(self.root, text="Angular Z").pack()
        self.slider_z = Scale(self.root, from_=-2.0, to=2.0, resolution=0.1, orient=HORIZONTAL)
        self.slider_z.pack()

        Button(self.root, text="STOP", command=self.stop).pack()

        self.update()
        self.root.mainloop()

    def update(self):
        self.twist.linear.x = self.slider_x.get()
        self.twist.linear.y = -self.slider_y.get()
        self.twist.angular.z = -self.slider_z.get()
        self.pub.publish(self.twist)
        self.root.after(100, self.update)

    def stop(self):
        self.slider_x.set(0)
        self.slider_y.set(0)
        self.slider_z.set(0)
        self.pub.publish(Twist())

if __name__ == "__main__":
    YoubotTeleop()

