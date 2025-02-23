import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import io
import base64

def map_callback(map_data):
    """
    Processes the OccupancyGrid map data, performs flipping and rotation,
    and saves the image to a location accessible by the Dash app.
    """
    try:
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y

        # Reshape the data into a 2D array
        map_array = np.array(map_data.data).reshape((height, width))

        # Reverse the array vertically and rotate 90 degrees
        map_array = np.flipud(map_array)
        map_array = np.rot90(map_array, k=4)

        # Normalize the data to 0-255 (uint8)
        map_array = 255 - (map_array * 255 / 100).astype(np.uint8)

        # Create an image from the array
        img = Image.fromarray(map_array, mode='L')

        buffer = io.BytesIO()
        img.save(buffer, format="png")
        encoded_image = buffer.getvalue()

        #Save image to a path, the path must be correct
        with open("/home/duc/Downloads/App MIR100/static/map_image.png", "wb") as fh:
            fh.write(encoded_image)

        rospy.loginfo("Saved rotated map image")

    except Exception as e:
        rospy.logerr(f"Error processing map data: {e}")

def listener():
    rospy.init_node('map_to_image', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()