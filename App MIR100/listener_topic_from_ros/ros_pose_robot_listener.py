# robot_position_map.py
import dash
from dash import dcc, html, Input, Output
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
import rospy
import tf
import numpy as np

# Khởi tạo ROS node
rospy.init_node('robot_position_map_node', anonymous=True)

# Khởi tạo ứng dụng Dash
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

# Biến toàn cục để lưu vị trí và hướng của robot
robot_position = (0, 0)  # Vị trí robot (x, y)
robot_orientation = 0  # Góc yaw của robot (radian)

# Layout của ứng dụng
app.layout = html.Div(
    [
        dcc.Graph(id="map-graph", style={"height": "600px", "width": "800px"}),
        dcc.Interval(id="interval-component", interval=1000, n_intervals=0),  # Cập nhật mỗi giây
    ],
    style={"padding": "20px", "background": "#ECF0F1"},
)

def get_robot_position():
    """
    Lấy vị trí và hướng của robot từ TF.
    """
    try:
        tf_listener = tf.TransformListener()
        (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        return (trans[0], trans[1]), tf.transformations.euler_from_quaternion(rot)[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None, None

def draw_robot(x, y, yaw):
    """
    Vẽ robot trên bản đồ sử dụng Plotly.
    """
    # Vẽ hình chữ nhật biểu thị thân robot
    rect_length = 0.6
    rect_width = 0.4
    corners = [
        (x + rect_length / 2 * np.cos(yaw) - rect_width / 2 * np.sin(yaw),
         y + rect_length / 2 * np.sin(yaw) + rect_width / 2 * np.cos(yaw)),
        (x + rect_length / 2 * np.cos(yaw) + rect_width / 2 * np.sin(yaw),
         y + rect_length / 2 * np.sin(yaw) - rect_width / 2 * np.cos(yaw)),
        (x - rect_length / 2 * np.cos(yaw) + rect_width / 2 * np.sin(yaw),
         y - rect_length / 2 * np.sin(yaw) - rect_width / 2 * np.cos(yaw)),
        (x - rect_length / 2 * np.cos(yaw) - rect_width / 2 * np.sin(yaw),
         y - rect_length / 2 * np.sin(yaw) + rect_width / 2 * np.cos(yaw))
    ]

    # Vẽ hình tam giác biểu thị hướng robot
    tri_side = 0.3
    triangle_points = [
        (x + tri_side * np.cos(yaw), y + tri_side * np.sin(yaw)),
        (x - tri_side / 2 * np.cos(yaw) + (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
         y - tri_side / 2 * np.sin(yaw) - (tri_side * np.sqrt(3) / 2) * np.cos(yaw)),
        (x - tri_side / 2 * np.cos(yaw) - (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
         y - tri_side / 2 * np.sin(yaw) + (tri_side * np.sqrt(3) / 2) * np.cos(yaw))
    ]

    # Tạo figure với Plotly
    fig = go.Figure()

    # Thêm hình chữ nhật
    fig.add_trace(go.Scatter(
        x=[corner[0] for corner in corners],
        y=[corner[1] for corner in corners],
        fill="toself",
        fillcolor="rgba(255, 0, 0, 0.4)",
        line=dict(color="rgba(255, 0, 0, 1)"),
        name="Robot Body"
    ))

    # Thêm hình tam giác
    fig.add_trace(go.Scatter(
        x=[point[0] for point in triangle_points],
        y=[point[1] for point in triangle_points],
        fill="toself",
        fillcolor="rgba(0, 0, 255, 0.7)",
        line=dict(color="rgba(0, 0, 255, 1)"),
        name="Robot Direction"
    ))

    # Cập nhật layout
    fig.update_layout(
        xaxis=dict(range=[x - 5, x + 5]),  # Giới hạn trục x
        yaxis=dict(range=[y - 5, y + 5]),  # Giới hạn trục y
        showlegend=False,
        margin=dict(l=0, r=0, t=0, b=0),
    )

    return fig

# Callback để cập nhật vị trí robot và vẽ lên bản đồ
@app.callback(
    Output("map-graph", "figure"),
    Input("interval-component", "n_intervals")
)
def update_map(n):
    global robot_position, robot_orientation

    # Lấy vị trí và hướng của robot từ TF
    position, orientation = get_robot_position()
    if position and orientation:
        robot_position = position
        robot_orientation = orientation

    # Vẽ robot trên bản đồ
    return draw_robot(robot_position[0], robot_position[1], robot_orientation)

if __name__ == "__main__":
    app.run_server(debug=True)