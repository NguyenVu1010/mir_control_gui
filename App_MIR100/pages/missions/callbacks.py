from dash import Output, Input
from dash import html
import dash_bootstrap_components as dbc
from dash_iconify import DashIconify
import requests
from .layout import mission_queue_layout

def get_mission_queue():
    url = "http://<robot-ip>/api/v2.0.0/missions"  # Thay <robot-ip> bằng IP thật của robot
    headers = {"Authorization": "Bearer <your-token>"}  # Nếu API cần token
    response = requests.get(url, headers=headers)

    if response.status_code == 200:
        return response.json()  # Trả về danh sách nhiệm vụ
    else:
        return []
def register_callbacks(app):
    @app.callback(
        Output("mission-table", "data"),
        Input("mission-interval", "n_intervals"),
    )
    def update_mission_table(_):
        missions = get_mission_queue()
        return mission_queue_layout(missions)
