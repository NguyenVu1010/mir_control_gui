import rospy
from ros_connect import RobotController
from thetastar import ThetaStarPathPlanner
import utils
import math
import reeds_shepp as rs
import turtle
import draw
from PIL import Image
def generate_path(pts, start_angle, end_angle):
    PATH = []
    
    # Thêm điểm đầu với góc đầu
    PATH.append((pts[0][0], pts[0][1], start_angle))
    
    for i in range(len(pts) - 1):
        dx = pts[i+1][0] - pts[i][0]
        dy = pts[i+1][1] - pts[i][1]
        theta = math.atan2(dy, dx)
        PATH.append((pts[i+1][0], pts[i+1][1], utils.rad2deg(theta)))
    
    # Thêm điểm cuối với góc cuối
    PATH.append((pts[-1][0], pts[-1][1], end_angle))
    
    return PATH
def main():
    global end_angle 
    rospy.init_node('robot_controller_node', anonymous=True)

    # Khởi tạo điều khiển và chờ dữ liệu ROS
    robot_controller = RobotController()

    # Chờ đến khi có vị trí robot
    while robot_controller.pose is None:
        rospy.loginfo("Waiting for AMCL pose...")
        rospy.sleep(0.5)

    x, y, theta = robot_controller.pose

    # Khởi tạo thuật toán Theta*
    path_planner = ThetaStarPathPlanner(
        size = (0.3,0.5)
    )
    # Chờ đến khi có dữ liệu bản đồ
    while path_planner.map_data is None:
        rospy.loginfo("Waiting for map data...")
        rospy.sleep(0.5)
    # Điểm bắt đầu và kết thúc
    start = (x, y)
    goal = (19.0, 13.0)  # Bạn có thể thay đổi mục tiêu tùy ý
    
    # Tìm đường
    pts = path_planner.theta_star(start, goal)
    
    if pts:
        PATH = []
        PATH=generate_path(pts,robot_controller.pose[2],120)
        # Tạo một màn hình Turtle
        screen = turtle.Screen()

        # Đặt kích thước màn hình nếu cần thiết
        screen.setup(width=600, height=600)

        # Đặt hình ảnh làm ảnh nền
        screen.bgpic("map_image.png")
        
        tesla = turtle.Turtle()
        tesla.speed(0) # 0: fast; 1: slow, 8.4: cool
        tesla.shape('arrow')
        tesla.resizemode('user')
        tesla.shapesize(1, 1)
        tesla.hideturtle()
        # draw vectors representing points in PATH
        for pt in PATH:
            draw.goto(tesla, pt)
            draw.vec(tesla)

        # draw all routes found
        # tesla.speed(0)
        # for i in range(len(PATH) - 1):
        #     paths = rs.get_all_paths(PATH[i], PATH[i+1])
            #print("path" + str(i) + ": " + str(paths) + "\n")

            # for path in paths:
            #     draw.set_random_pencolor(tesla)
            #     draw.goto(tesla, PATH[i])
            #     draw.draw_path(tesla, path)

        # draw shortest route
        tesla.pencolor(1, 0, 0)
        tesla.pensize(3)
        tesla.speed(10)
        draw.goto(tesla, PATH[0])
        path_length = 0
        for i in range(len(PATH) - 1):
            path = rs.get_optimal_path(PATH[i], PATH[i+1])
            print(str(path))
            path_length += rs.path_length(path)
            draw.draw_path(tesla, path)

        print("Shortest path length: {} px.".format(int(draw.scale(path_length))))
        canvas = screen.getcanvas()

        # Lưu ảnh chụp màn hình từ canvas
        canvas.postscript(file="drawing.eps")

        # Sử dụng PIL để chuyển từ EPS sang PNG
        img = Image.open("drawing.eps")
        img.save("drawing.png", "PNG")
        turtle.done()

    rospy.spin()

if __name__ == '__main__':
    main()
