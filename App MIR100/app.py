# app.py
import dash
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
import dash_daq as daq
from components import LoginPage, ChangePasswordPage, Sidebar, StatusBar, MapSection
from utils.data import authenticate, user_credentials, update_password
import time  # Thêm thư viện time
import datetime

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

# Instantiate components
login_page = LoginPage()
change_password_page = ChangePasswordPage()
sidebar = Sidebar()
status_bar = StatusBar()
map_section = MapSection()

# Main app layout (initially set to the login layout)
app.layout = html.Div(id="app-container", children=[login_page.layout])

# Callback for the login button
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
                status_bar.create_status_bar(),
                sidebar.create_sidebar(),
                map_section.create_map_section(),
            ],
            style={"background": "#BDC3C7", "height": "100vh", "overflow": "hidden"},
        )
    else:
        return html.Div([login_page.layout, html.Div("Login Failed", style={"color": "red"})])

# Callback for updating content based on sidebar selection
@app.callback(
    Output("content-area", "children"),
    Input("sidebar-nav", "n_clicks"),
    State("sidebar-nav", "children"),
    prevent_initial_call=True
)
def update_content(n_clicks, nav_links):
    ctx = dash.callback_context
    if not ctx.triggered:
        return html.P("Select a function from the sidebar")

    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    link_id = next((i for i in nav_links[2]["props"]["children"] if i["props"]["id"] == button_id), None)

    if link_id:
        if link_id["props"]["id"] == "missions-link":
            return html.Div("Missions content here")
        elif link_id["props"]["id"] == "maps-link":
            return html.Div("Maps content here")
        elif link_id["props"]["id"] == "change-password-link":
            return change_password_page.layout  # Render change password layout
        else:
            return html.Div(f"Content for {link_id['props']['id']}")
    else:
        return html.Div("Nothing to show.")

# Callback for password update
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
        username = list(user_credentials.keys())[0]  # Get the username (in this example, it's "admin")
        if update_password(username, new_password):
            return html.Div("Password updated successfully!", style={"color": "green"})
        else:
            return html.Div("Failed to update password.", style={"color": "red"})
    else:
        return html.Div("Passwords do not match.", style={"color": "red"})

# Callback for joystick popup
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
                daq.Joystick(id="joystick", label="Joystick", angle=0, force=0, size=150)  # Adjusted size
            ),
            dbc.ModalFooter(
                dbc.Button("Close", id="close-joystick-btn", className="ms-auto", n_clicks=0)
            ),
        ],
        id="joystick-modal",
        is_open=True,
        centered=True,  # Center the modal
        size="lg",  # Adjust size as needed
    )

# Callback for closing joystick
@app.callback(
    Output("joystick-modal", "is_open"),
    Input("close-joystick-btn", "n_clicks"),
    State("joystick-modal", "is_open"),
    prevent_initial_call=True,
)
def close_joystick(n_clicks, is_open):
    return not is_open

# Callback to handle language change
@app.callback(
    Output('map_section', 'children'),
    Input('language-dropdown', 'value')
)
def change_language(language):
    # Logic to change text based on selected language
    # For example, use a dictionary to store translations
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
            html.Div(id="content-area"),  # Placeholder for content based on sidebar selection
        ],
        style={
            "padding": "20px",
            "flex": "1",
            "background": "#ECF0F1",
            "marginLeft": "250px",
            "marginTop": "50px",
        },
    )

# Callback to update map image
@app.callback(
    Output("map-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_map_image(n):
    # Add a timestamp to the image URL to force refresh
    return f"/static/map_image.png?{int(time.time())}"

@app.callback(
    Output("map-graph", "figure"),
    Input(component_id='interval-component', component_property='n_intervals')
)
def update_graph_figure(n):
    return map_section.create_figure()

@app.callback(
    Output("lidar-image", "src"),
    Input(component_id='interval-component', component_property='n_intervals')
)
def update_map_image(n):
    time_img = datetime.now()
    return "static/map_image.png?{}".format(time_img) #Force server to refresh img

if __name__ == "__main__":
    app.run_server(debug=True)