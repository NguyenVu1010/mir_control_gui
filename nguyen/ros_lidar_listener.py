import rospy
from sensor_msgs.msg import LaserScan
from PIL import Image, ImageDraw
import io
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped

# Image dimensions
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600
LIDAR_RANGE = 10  # Adjust this to lidar range for display

def process_lidar_data(msg, tf_listener, is_back):
    points = []
    frame_id = '/front_laser_link' if not is_back else '/back_laser_link'

    for i in range(len(msg.ranges)):
        r = msg.ranges[i]
        angle = i * msg.angle_increment + msg.angle_min

        if r < msg.range_max and r > msg.range_min and r < LIDAR_RANGE:
            x = r * np.cos(angle)
            y = r * np.sin(angle)

            # Create PointStamped in laser frame
            point_stamped = PoseStamped()
            point_stamped.header.frame_id = frame_id
            point_stamped.header.stamp = rospy.Time(0)  # Use latest transform
            point_stamped.pose.position.x = x
            point_stamped.pose.position.y = y
            point_stamped.pose.position.z = 0.0  # Assuming 2D data
            point_stamped.pose.orientation.w = 1.0  # A valid orientation is required

            try:
                # Transform point to /map frame
                transformed_point = tf_listener.transformPose("/map", point_stamped)

                # Append transformed point to the list
                points.append((transformed_point.pose.position.x, transformed_point.pose.position.y))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF error: {e}")
                continue

    return points

def create_lidar_image(points):
    """
    Creates an image representing the lidar data with a transparent background.
    """
    img = Image.new('RGBA', (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))  # Create a transparent image
    draw = ImageDraw.Draw(img)

    if not points:
        return img  # Return a transparent image if no points are available

    # Find min and max for scaling to range to 0-1
    all_x = [p[0] for p in points]
    all_y = [p[1] for p in points]
    min_x = min(all_x)
    max_x = max(all_x)
    min_y = min(all_y)
    max_y = max(all_y)

    # Scale and translate points to image coordinates
    for point_x, point_y in points: #Scale and transform the points
        # Scale and translate the point value to make it in range
        px = int(((point_x - min_x) / (max_x - min_x)) * IMAGE_WIDTH)
        py = int(((point_y - min_y) / (max_y - min_y)) * IMAGE_HEIGHT)

        #Plot white point and not black (0,0,0)
        draw.point((px, py), fill=(255, 0, 0))

    return img

def scan_callback(msg, tf_listener, topic_name):
    try:
        #Use lambda function call from main, by calling its name it create a
        points_b = process_lidar_data(msg, tf_listener, (topic_name == "/b_scan"))

        #create a image
        img = create_lidar_image(points_b)
        img.save(f"/home/duc/Downloads/App MIR100/static/{topic_name}_image.png")

        points_f = process_lidar_data(msg, tf_listener, (topic_name == "/f_scan"))

        #create a image
        img = create_lidar_image(points_b)
        img.save(f"/home/duc/Downloads/App MIR100/static/{topic_name}_image.png")

        rospy.loginfo(f"Lidar image {topic_name} created")

    except Exception as e:
        rospy.logerr(f"Error processing {topic_name} data: {e}")

def listener():
    rospy.init_node('lidar_to_image', anonymous=True)
    tf_listener = tf.TransformListener() #This create the class so you can get

    rospy.Subscriber("/f_scan", LaserScan, lambda msg: scan_callback(msg, tf_listener, "/f_scan"))
    rospy.Subscriber("/b_scan", LaserScan, lambda msg: scan_callback(msg, tf_listener, "/b_scan"))
    rospy.spin()

if __name__ == '__main__':
    listener()