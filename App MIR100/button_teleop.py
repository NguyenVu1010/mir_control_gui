# button_teleop.py
import rospy
from geometry_msgs.msg import Twist
from dash import html

class ButtonTeleop:
    def __init__(self, pub, linear_speed=0.5, angular_speed=0.3):
        self.pub = pub
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.twist = Twist()

    def move_forward(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0
        self.publish_velocity()

    def move_backward(self):
        self.twist.linear.x = -self.linear_speed
        self.twist.angular.z = 0.0
        self.publish_velocity()

    def turn_left(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = self.angular_speed
        self.publish_velocity()

    def turn_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -self.angular_speed
        self.publish_velocity()

    def move_forward_left(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed
        self.publish_velocity()

    def move_forward_right(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = -self.angular_speed
        self.publish_velocity()

    def move_backward_left(self):
        self.twist.linear.x = -self.linear_speed
        self.twist.angular.z = self.angular_speed
        self.publish_velocity()

    def move_backward_right(self):
        self.twist.linear.x = -self.linear_speed
        self.twist.angular.z = -self.angular_speed
        self.publish_velocity()

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publish_velocity()

    def publish_velocity(self):
        self.pub.publish(self.twist)

    def create_buttons(self):
         return html.Div(
            [
                html.Div(
                    [
                        html.Button("↑", id="forward-button", n_clicks=0, style={'width': '50px', 'height': '50px'}),
                    ],
                    style={"gridColumn": "2", "gridRow": "1", "display": "flex", "justifyContent": "center"}
                ),
                 html.Div(
                    [
                        html.Button("↖", id="forward-left-button", n_clicks=0, style={'width': '50px', 'height': '50px'}),
                    ],
                    style={"gridColumn": "1", "gridRow": "1", "display": "flex", "justifyContent": "center"}
                ),
                html.Div(
                    [
                        html.Button("↗", id="forward-right-button", n_clicks=0, style={'width': '50px', 'height': '50px'}),
                    ],
                    style={"gridColumn": "3", "gridRow": "1", "display": "flex", "justifyContent": "center"}
                ),
                html.Div(
                    [
                        html.Button("←", id="left-button", n_clicks=0, style={'width': '50px', 'height': '50px'}),
                    ],
                    style={"gridColumn": "1", "gridRow": "2", "display": "flex", "justifyContent": "center"}
                ),
                 html.Div(
                    [
                         html.Button("Stop", id="stop-button", n_clicks=0, style={'width': '50px', 'height': '50px'})
                    ],
                    style={"gridColumn": "2", "gridRow": "2", "display": "flex", "justifyContent": "center"}
                ),
                html.Div(
                    [
                        html.Button("→", id="right-button", n_clicks=0, style={'width': '50px', 'height': '50px'}),
                    ],
                     style={"gridColumn": "3", "gridRow": "2", "display": "flex", "justifyContent": "center"}
                ),
                 html.Div(
                    [
                        html.Button("↙", id="back-left-button", n_clicks=0, style={'width': '50px', 'height': '50px'}),
                    ],
                    style={"gridColumn": "1", "gridRow": "3", "display": "flex", "justifyContent": "center"}
                ),
                  html.Div(
                    [
                        html.Button("↘", id="back-right-button", n_clicks=0, style={'width': '50px', 'height': '50px'}),
                    ],
                    style={"gridColumn": "3", "gridRow": "3", "display": "flex", "justifyContent": "center"}
                ),
                html.Div(
                    [
                        html.Button("↓", id="backward-button", n_clicks=0, style={'width': '50px', 'height': '50px'}),
                    ],
                    style={"gridColumn": "2", "gridRow": "3", "display": "flex", "justifyContent": "center"}
                ),

            ],
            style={"display": "grid",
                    "gridTemplateColumns": "auto auto auto",
                   "padding": "10px"}
        )