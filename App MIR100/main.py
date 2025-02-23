import rospy
from listener_topic_from_ros.ros_lidar_listener import *
from listener_topic_from_ros.ros_map_listener import *  

def main():
    rospy.init_node('main_node', anonymous=True)
    rospy.Subscriber("/f_scan", LaserScan,lambda msg: scan_callback(msg, tf_listener, ("/f_scan")))
    rospy.Subscriber("/b_scan", LaserScan, lambda msg: scan_callback(msg, tf_listener, ("/b_scan")))
    tf_listener = tf.TransformListener()
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    main()