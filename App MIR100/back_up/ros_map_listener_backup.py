import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import io

def map_callback(map_data):
    try:
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y
        map_array = np.array(map_data.data).reshape((height, width))
        map_array = np.flipud(map_array)
        map_array = np.rot90(map_array, k=4)
        map_array = (map_array * 255 / 100).astype(np.uint8) 
        map_array = 255 - map_array
        img = Image.fromarray(map_array, mode='L')
        # img = img.resize((IMAGE_WIDTH, IMAGE_HEIGHT), Image.LANCZOS) #resize the image by LANCZO
        buffer = io.BytesIO()
        img.save(buffer, format="png")
        encoded_image = buffer.getvalue()
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