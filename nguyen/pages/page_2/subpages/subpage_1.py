from dash import html

layout = html.Div([
    html.H1("Trang 1 - Điều khiển robot"),
    html.P("Đây là giao diện điều khiển chính của robot."),
    html.Button("Bắt đầu", id="start-btn", n_clicks=0)
])
