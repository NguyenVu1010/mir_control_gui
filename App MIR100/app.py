# app.py
import dash
from dash import dcc, html, Input, Output, State, callback, callback_context, no_update
import dash_bootstrap_components as dbc
import dash_daq as daq
from components import LoginPage, ChangePasswordPage, Sidebar, StatusBar, MapSection
from utils.data import authenticate, user_credentials, update_password
import time
import random
import rospy
from components.draw_mode import create_draw_mode_layout
import plotly.graph_objects as go
from draw_mode_callbacks import *  # Import the callbacks

# Khởi tạo ứng dụng Dash
app = dash.Dash(
    __name__,
    external_stylesheets=[
        dbc.themes.BOOTSTRAP,
        "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css",
    ],
    suppress_callback_exceptions=True,
    external_scripts=["assets/script.js"]
)

# Khởi tạo các component
login_page = LoginPage()
change_password_page = ChangePasswordPage()
sidebar = Sidebar()
status_bar = StatusBar()
map_section = MapSection()

# Định nghĩa layout chính của ứng dụng
app.layout = html.Div(
    [
        dcc.Location(id='url', refresh=False),
        html.Div(id="app-container", children=[login_page.layout])
    ]
)

# Callback xử lý đăng nhập
@app.callback(
    Output("app-container", "children"),
    Input("login-button", "n_clicks"),
    State("username", "value"),
    State("password", "value"),
    prevent_initial_call=True
)
def login(n_clicks, username, password):
    if authenticate(username, password):
        return html.Div(
            [
                dcc.Location(id='url', refresh=False),
                status_bar.create_status_bar(),
                sidebar.create_sidebar(),
                html.Div(id="page-content", style={"marginLeft": "250px"}),
            ],
            style={"background": "#BDC3C7", "height": "100vh", "overflow": "hidden"},
        )
    else:
        return html.Div([login_page.layout, html.Div("Login Failed", style={"color": "red"})])

# Callback cập nhật nội dung trang dựa trên URL
@app.callback(
    Output('page-content', 'children'),
    Input('url', 'pathname')
)
def display_page(pathname):
    if pathname == '/draw-mode':
        return create_draw_mode_layout()
    elif pathname == '/maps':
        return map_section.create_map_section()
    elif pathname == '/change-password':
        return change_password_page.layout
    else:
        return html.Div([
            status_bar.create_status_bar(),
            map_section.create_map_section()
        ])

# Callback cập nhật mật khẩu
@app.callback(
    Output("password-status", "children"),
    Input("update-password-button", "n_clicks"),
    State("new-password", "value"),
    State("confirm-password", "value"),
    prevent_initial_call=True
)
def update_password_callback(n_clicks, new_password, confirm_password):
    if new_password == confirm_password:
        global user_credentials
        username = list(user_credentials.keys())[0]
        if update_password(username, new_password):
            return html.Div("Password updated successfully!", style={"color": "green"})
        else:
            return html.Div("Failed to update password.", style={"color": "red"})
    else:
        return html.Div("Passwords do not match.", style={"color": "red"})

# Callback hiển thị popup joystick
@app.callback(
    Output("joystick-popup-container", "children"),
    Input("open-joystick-btn", "n_clicks"),
    prevent_initial_call=True,
)
def open_joystick(n_clicks):
    return dbc.Modal(
        [
            dbc.ModalHeader(dbc.ModalTitle("Joystick Control")),
            dbc.ModalBody(
                daq.Joystick(id="joystick", label="Joystick", angle=0, force=0, size=150)
            ),
            dbc.ModalFooter(
                dbc.Button("Close", id="close-joystick-btn", className="ms-auto", n_clicks=0)
            ),
        ],
        id="joystick-modal",
        is_open=True,
        centered=True,
        size="lg",
    )

# Callback đóng popup joystick
@app.callback(
    Output("joystick-modal", "is_open"),
    Input("close-joystick-btn", "n_clicks"),
    State("joystick-modal", "is_open"),
    prevent_initial_call=True
)
def close_joystick(n_clicks, is_open):
    return not is_open

# Callback thay đổi ngôn ngữ
@app.callback(
    Output('map_section', 'children'),
    Input('language-dropdown', 'value')
)
def change_language(language):
    translations = {
        'en': {'title': 'Main Floor', 'map': 'Edit and draw the map', 'ready': 'The map is ready for your work.'},
        'vi': {'title': 'Tầng Chính', 'map': 'Chỉnh sửa và vẽ bản đồ', 'ready': 'Bản đồ đã sẵn sàng để làm việc.'},
        'es': {'title': 'Piso Principal', 'map': 'Edita y dibuja el mapa', 'ready': 'El mapa está listo para tu trabajo.'},
    }
    translation = translations.get(language, translations['en'])
    return html.Div(
        [
            html.H3(translation['title'], className="mb-3", style={"color": "#2C3E50"}),
            html.P(translation['map'], className="text-muted"),
            html.Img(src="/path/to/save/map_image.png", style={"width": "100%", "border": "2px solid #34495E"}),
            html.P(translation['ready'], className="text-info mt-2"),
            html.Div(id="content-area"),
        ],
        style={
            "padding": "20px",
            "flex": "1",
            "background": "#ECF0F1",
            "marginLeft": "250px",
            "marginTop": "50px",
        },
    )

# Callback cập nhật hình ảnh bản đồ
@app.callback(
    Output("map-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_map_image(n):
    timestamp = int(time.time())
    return f"/static/map_image.png?{timestamp}"

# Callback cập nhật hình ảnh lidar
@app.callback(
    Output("lidar-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_lidar_image(n):
    timestamp = int(time.time())
    return f"/static/lidar_image.png?{timestamp}"

# Callback cập nhật đồ thị bản đồ
@app.callback(
    Output("map-graph", "figure"),
    Input(component_id='interval-component', component_property='n_intervals')
)
def update_graph_figure(n):
    return map_section.create_figure()

# Callback cập nhật hình ảnh lidar (trước và sau)
@app.callback(
    [Output("lidar-f-image", "src"), Output("lidar-b-image", "src")],
    Input("interval-component", "n_intervals")
)
def update_lidar_images(n):
    timestamp = int(time.time())
    return (
        f"/static/_scan_image.png?{timestamp}",
        f"/static/b_scan_image.png?{timestamp}"
    )

# Callback cập nhật hình ảnh đường đi
@app.callback(
    [Output("path-image", "src")],
    Input("interval-component", "n_intervals")
)
def update_path_image(n):
    random_value = random.randint(1, 100000)
    return (
        f"/static/path_image.png?random={random_value}",
    )

# Callback cập nhật hình ảnh bản đồ trong chế độ vẽ
@app.callback(
    Output("map-image-draw-mode", "src"),
    Input("interval-component", "n_intervals")
)
def update_map_image_draw_mode(n):
    timestamp = int(time.time())
    return f"/static/map_image.png?{timestamp}"

# Chạy ứng dụng
if __name__ == "__main__":
    rospy.init_node('dash_app', anonymous=True)
    app.run_server(debug=True)