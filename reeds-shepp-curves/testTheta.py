import rospy
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid
from scipy.interpolate import splprep, splev

class ThetaStarPathPlanner:
    def __init__(self, robot_radius=0.3):
        rospy.init_node("theta_star_path_planner", anonymous=True)
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.robot_radius = robot_radius  
        self.valid_map = None  

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.sleep(2)  

    def map_callback(self, msg):
        """ Nhận dữ liệu bản đồ và tạo mảng hợp lệ """
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.generate_valid_map()

    def generate_valid_map(self):
        """ Tạo bản đồ hợp lệ để kiểm tra va chạm nhanh hơn """
        height, width = self.map_data.shape
        self.valid_map = np.zeros((height, width), dtype=bool)
        radius_px = int(self.robot_radius / self.resolution)

        for y in range(height):
            for x in range(width):
                if self.map_data[y, x] > 50:  
                    for dy in range(-radius_px, radius_px + 1):
                        for dx in range(-radius_px, radius_px + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                self.valid_map[ny, nx] = True  
        
    def world_to_map(self, x, y):
        """ Chuyển tọa độ thực tế sang tọa độ bản đồ """
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        """ Chuyển tọa độ bản đồ sang tọa độ thực tế """
        x = mx * self.resolution + self.origin[0]
        y = my * self.resolution + self.origin[1]
        return x, y

    def is_valid(self, mx, my):
        """ Kiểm tra xem điểm có hợp lệ không """
        if 0 <= mx < self.map_data.shape[1] and 0 <= my < self.map_data.shape[0]:
            return not self.valid_map[my, mx]
        return False

    def line_of_sight(self, p1, p2):
        """ Kiểm tra xem có đường đi thẳng giữa hai điểm không """
        x0, y0 = p1
        x1, y1 = p2
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
        err = dx - dy

        while (x0, y0) != (x1, y1):
            if not self.is_valid(x0, y0):
                return False
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return True

    def theta_star(self, start, goal):
        """ Thuật toán Theta* tìm đường đi """
        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        start_node = self.world_to_map(*start)
        goal_node = self.world_to_map(*goal)

        open_list = []
        heapq.heappush(open_list, (0, start_node))
        came_from = {start_node: None}
        g_score = {start_node: 0}
        f_score = {start_node: heuristic(start_node, goal_node)}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal_node:
                path = []
                while current:
                    path.append(self.map_to_world(*current))
                    current = came_from[current]
                return path[::-1]

            neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0),  
                         (1, 1), (1, -1), (-1, 1), (-1, -1)]  

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                if self.is_valid(*neighbor):
                    new_g_score = g_score[current] + heuristic(current, neighbor)

                    if neighbor in came_from and new_g_score >= g_score.get(neighbor, float('inf')):
                        continue  

                    # Nếu có đường đi thẳng từ cha của current đến neighbor -> bỏ qua current
                    if came_from[current] and self.line_of_sight(came_from[current], neighbor):
                        parent = came_from[current]
                        new_g_score = g_score[parent] + heuristic(parent, neighbor)
                    else:
                        parent = current

                    came_from[neighbor] = parent
                    g_score[neighbor] = new_g_score
                    f_score[neighbor] = new_g_score + heuristic(neighbor, goal_node)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None  

    



