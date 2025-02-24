import rospy
import tf
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from listener_topic_from_ros.ros_lidar_listener import *
from listener_topic_from_ros.ros_map_listener import *
from listener_topic_from_ros.ros_path_listener import *

def main():
    rospy.init_node('main_node', anonymous=True)
    tf_listener = tf.TransformListener()  
    rospy.loginfo("Waiting for map data...")
    try:
        map_data = rospy.wait_for_message("/map", OccupancyGrid, timeout=10) 
        map_info_callback(map_data)
        rospy.loginfo("Map data received.")
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout while waiting for map data: {e}")
        return  
    rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, path_callback)
    rospy.Subscriber("/f_scan", LaserScan, lambda msg: scan_callback(msg, tf_listener, "/f_scan"))
    rospy.Subscriber("/b_scan", LaserScan, lambda msg: scan_callback(msg, tf_listener, "/b_scan"))
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    
    rospy.spin() 

if __name__ == '__main__':
    main()