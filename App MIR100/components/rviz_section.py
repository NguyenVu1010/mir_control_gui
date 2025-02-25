# rviz_section.py
from dash import html, dcc, callback, Input, Output, State
import dash_bootstrap_components as dbc
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
import plotly.graph_objects as go
from components.goal import Goal
import numpy as np
import dash

class RVizSection:
    def __init__(self, goal_topic="/move_base_simple/goal"):
        self.goal_topic = goal_topic
        try:
            self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        except rospy.exceptions.ROSException as e:
            print(f"Error connecting to ROS: {e}")
            self.goal_pub = None 

    def create_rviz_section(self):
        layout = html.Div(
            [
                html.H3("RViz Interface", className="mb-3", style={"color": "#2C3E50"}),
                dbc.Row(
                    [
                        dbc.Col(
                            html.Div(
                                [
                                    html.Button("Send Goal", id="send-goal-btn", className="btn btn-primary me-2"),
                                    html.Button("2D Nav Goal", id="nav-goal-btn", className="btn btn-secondary"),
                                ],
                                className="mb-3"
                            ),
                            width=12,
                        ),
                    ]
                ),
                # Container for map and lidar images (giống với map_section)
                html.Div(
                    [
                        # Map image
                        html.Img(
                            id="map-image",
                            src="/static/map_image.png",
                            style={
                                "width": "800px",  # Fixed size
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",  # Ensure the image is not distorted
                                "position": "absolute",  # Position the map image
                                "z-index": "1",  # Map image is below lidar image
                            },
                        ),
                        # Lidar image (front scan)
                        html.Img(
                            id="lidar-f-image",
                            src="/static/f_scan_image.png",
                            style={
                                "width": "800px",  # Same size as map image
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  # Position the lidar image
                                "top": "0",  # Align with map image
                                "left": "0",  # Align with map image
                                "z-index": "2",  # Lidar image is above map image
                            },
                        ),
                        # Lidar image (back scan)
                        html.Img(
                            id="lidar-b-image",
                            src="/static/b_scan_image.png",
                            style={
                                "width": "800px",  # Same size as map image
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  # Position the lidar image
                                "top": "0",  # Align with map image
                                "left": "0",  # Align with map image
                                "z-index": "2",  # Lidar image is above map image
                            },
                        ),
                        # Path image
                        html.Img(
                            id="path-image",
                            src="/static/path_image.png",
                            style={
                                "width": "800px",  # Same size as map image
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  # Position the lidar image
                                "top": "0",  # Align with map image
                                "left": "0",  # Align with map image
                                "z-index": "3",  # Lidar image is above map image
                            },
                        ),
                    ],
                    style={
                        "position": "relative",  # Container for absolute positioning
                        "width": "800px",  # Same size as images
                        "height": "600px",
                    },
                ),
                html.P("The map is ready for your work.", className="text-info mt-2"),
                html.Div(id="content-area"),  # Placeholder for content based on sidebar selection
                dcc.Interval(
                    id='interval-component',
                    interval=1*1000,  # Update every second
                    n_intervals=0
                ),
                # Modal for entering goal coordinates
                dbc.Modal(
                    [
                        dbc.ModalHeader(dbc.ModalTitle("Enter Goal Coordinates")),
                        dbc.ModalBody(
                            dbc.Form(
                                [
                                    dbc.Row(
                                        [
                                            dbc.Col(dbc.Label("X:", html_for="goal-x")),
                                            dbc.Col(dbc.Input(type="number", id="goal-x", placeholder="X Coordinate")),
                                        ],
                                        className="mb-3",
                                    ),
                                    dbc.Row(
                                        [
                                            dbc.Col(dbc.Label("Y:", html_for="goal-y")),
                                            dbc.Col(dbc.Input(type="number", id="goal-y", placeholder="Y Coordinate")),
                                        ],
                                        className="mb-3",
                                    ),
                                    dbc.Row(
                                        [
                                            dbc.Col(dbc.Label("Z:", html_for="goal-z")),
                                            dbc.Col(dbc.Input(type="number", id="goal-z", placeholder="Z Coordinate", value=0)),
                                        ],
                                        className="mb-3",
                                    ),
                                    dbc.Row(
                                        [
                                            dbc.Col(dbc.Label("W:", html_for="goal-w")),
                                            dbc.Col(dbc.Input(type="number", id="goal-w", placeholder="W Orientation", value=1)),
                                        ],
                                        className="mb-3",
                                    ),
                                ]
                            )
                        ),
                        dbc.ModalFooter(
                            [
                                dbc.Button("Close", id="close-goal-modal", className="ms-auto"),
                                dbc.Button("Send", id="send-goal-modal-btn", color="primary", className="ms-2"),
                            ]
                        ),
                    ],
                    id="goal-modal",
                    is_open=False,
                ),
                dcc.Store(id="latest-goal", data=Goal().to_dict()),  # Store for the goal
                html.Div(id="goal-status"),  # For status messages
            ],
            style={
                "padding": "20px",
                "flex": "1",
                "background": "#ECF0F1",
                "marginLeft": "250px",
                "marginTop": "50px",
            },
        )
        return layout

    def publish_goal(self, x, y, z, w):
        if self.goal_pub:
            try:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"  # Or your map frame

                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = float(z)

                pose.pose.orientation.w = float(w)  # Keep orientation only on w axis

                self.goal_pub.publish(pose)
                return "Goal published successfully!"
            except Exception as e:
                return f"Error publishing goal: {e}"
        else:
            return "Goal publisher not initialized."

# Define the callbacks outside the class
@callback(
    Output("goal-modal", "is_open"),
    [Input("send-goal-btn", "n_clicks"), Input("close-goal-modal", "n_clicks"), Input("send-goal-modal-btn", "n_clicks")],
    [State("goal-modal", "is_open")],
    prevent_initial_call=True,
)
def toggle_modal(n1, n2, n3, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return False
    else:
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]

    if button_id == "send-goal-btn":
        return True
    elif button_id == "close-goal-modal" or  button_id == "send-goal-modal-btn":
        return False
    return is_open

@callback(
    [Output("goal-status", "children"), Output("latest-goal", "data")],
    Input("send-goal-modal-btn", "n_clicks"),
    [
        State("goal-x", "value"),
        State("goal-y", "value"),
        State("goal-z", "value"),
        State("goal-w", "value"),
        State("latest-goal", "data")
    ],
    prevent_initial_call=True,
)
def send_goal_coordinates(n_clicks, x, y, z, w, latest_goal):
    if n_clicks:
        rviz_section = RVizSection()
        status = rviz_section.publish_goal(x, y, z, w)
        goal = Goal(x, y, z, w)
        latest_goal = goal.to_dict()
        return status, latest_goal
    return dash.no_update, dash.no_update

@callback(
    Output("map-graph", "figure"),
    Input("interval-component", "n_intervals")
)
def update_graph_figure(n):
    rviz_section = RVizSection()
    return rviz_section.create_figure()

def create_rviz_section():
    return RVizSection().create_rviz_section()