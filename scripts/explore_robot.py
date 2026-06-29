#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

def move_robot():
    rospy.init_node('auto_explorer')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Sequence of (linear_x, angular_z, duration_sec)
    # A simple path to explore the 'lar' world
    trajectory = [
        (0.5, 0.0, 15.0),   # move forward
        (0.0, 0.5, 3.14),   # turn left
        (0.5, 0.0, 10.0),   # move forward
        (0.0, -0.5, 3.14),  # turn right
        (0.5, 0.0, 15.0),   # move forward
        (0.0, 0.5, 3.14),   # turn left
        (0.5, 0.0, 10.0),   # move forward
        (0.0, 0.5, 3.14),   # turn left
        (0.5, 0.0, 15.0),   # move forward
        (0.0, 0.0, 1.0)     # stop
    ]

    for v, w, t in trajectory:
        rospy.loginfo(f"Command: v={v}, w={w}, duration={t}")
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < t and not rospy.is_shutdown():
            pub.publish(cmd)
            rate.sleep()

    # Stop at the end
    cmd = Twist()
    pub.publish(cmd)
    rospy.loginfo("Exploration finished")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
