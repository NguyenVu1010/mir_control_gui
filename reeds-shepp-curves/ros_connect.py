import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
from tf.transformations import euler_from_quaternion

class RobotController:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.pose = None
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.datafortheta = None
    def move_robot(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)

    def amcl_pose_callback(self, msg):
        msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.pose = (x, y, theta)

