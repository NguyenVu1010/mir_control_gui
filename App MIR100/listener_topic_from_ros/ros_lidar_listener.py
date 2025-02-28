# ros_lidar_listener.py
import rospy
from sensor_msgs.msg import LaserScan
from PIL import Image, ImageDraw
import numpy as np
import tf
from geometry_msgs.msg import PointStamped

image_path = "/home/duc/Downloads/App MIR100/static/map_image.png"
try:
    img = Image.open(image_path)
    IMAGE_WIDTH, IMAGE_HEIGHT = img.size
    print("Map image successfully loaded.")
except FileNotFoundError:
    print(f"Error: Could not open image file at {image_path}")
    IMAGE_WIDTH, IMAGE_HEIGHT = 500, 500  
    img = Image.new('RGBA', (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
except Exception as e:
    print(f"Error opening image {image_path}: {e}")
    IMAGE_WIDTH, IMAGE_HEIGHT = 500, 500
    img = Image.new('RGBA', (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))

LIDAR_RANGE = 50  # Maximum range of the LiDAR to consider
POINT_SIZE = 1   # Size of the point to draw on the image
MANUAL_SCALE_FACTOR = 0.9  # Tweak this value to fine-tune scaling

def process_lidar_data(msg, tf_listener, is_back):
    points = []
    frame_id = "back_laser_link" if is_back else "front_laser_link"

    for i in range(len(msg.ranges)):
        r = msg.ranges[i]
        angle = i * msg.angle_increment + msg.angle_min

        if msg.range_min < r < msg.range_max and r < LIDAR_RANGE:
            x = r * np.cos(angle)
            y = r * np.sin(angle)

            point_stamped = PointStamped()
            point_stamped.header.frame_id = frame_id
            point_stamped.header.stamp = msg.header.stamp
            point_stamped.point.x = x
            point_stamped.point.y = y

            try:
                tf_listener.waitForTransform("map", frame_id, msg.header.stamp, rospy.Duration(1))
                if tf_listener.canTransform("map", frame_id, msg.header.stamp):
                    transformed_point = tf_listener.transformPoint("map", point_stamped)
                    points.append((transformed_point.point.x, transformed_point.point.y))
                else:
                    rospy.logwarn("Cannot transform point: Transformation not available.")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF error: {e}")
                continue
    return points

def create_lidar_image(points):
    img = Image.new('RGBA', (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    if not points:
        return img

    all_x = [p[0] for p in points]
    all_y = [p[1] for p in points]

    min_x = min(all_x)
    max_x = max(all_x)
    min_y = min(all_y)
    max_y = max(all_y)

    width = max_x - min_x
    height = max_y - min_y

    if width == 0 or height == 0:
        return img

    # Calculate a scaling factor that considers the map image size
    max_dimension = max(width, height)  
    scale = min(IMAGE_WIDTH, IMAGE_HEIGHT) / max_dimension * MANUAL_SCALE_FACTOR 

    # Center the points within the image.
    offset_x = (IMAGE_WIDTH / scale - width) / 2
    offset_y = (IMAGE_HEIGHT / scale - height) / 2

    for point_x, point_y in points:
        px = int((point_x - min_x + offset_x) * scale)
        py = int(IMAGE_HEIGHT - (point_y - min_y + offset_y) * scale)
        draw.ellipse((px - POINT_SIZE, py - POINT_SIZE, px + POINT_SIZE, py + POINT_SIZE), fill=(255, 0, 0))
    return img

def scan_callback(msg, tf_listener, topic_name):
    try:
        points = process_lidar_data(msg, tf_listener, is_back=(topic_name == "/b_scan"))
        img_output = create_lidar_image(points)
        output_path = f"/home/duc/Downloads/App MIR100/static/{topic_name.split('/')[-1]}_image.png"
        img_output.save(output_path)
        rospy.loginfo(f"Lidar image for {topic_name} created and saved to {output_path}")

    except Exception as e:
        rospy.logerr(f"Error processing {topic_name} data: {e}")

def listener():
    rospy.init_node('lidar_to_image', anonymous=True)
    tf_listener = tf.TransformListener()
    rospy.Subscriber("/f_scan", LaserScan, lambda msg: scan_callback(msg, tf_listener, "/f_scan"))
    rospy.Subscriber("/b_scan", LaserScan, lambda msg: scan_callback(msg, tf_listener, "/b_scan"))
    rospy.spin()

if __name__ == '__main__':
    listener()