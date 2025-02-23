from dash import html, dcc
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output
from .subpages import subpage_1, subpage_2, subpage_3, subpage_4
from main import app  

# Thanh taskbar dọc (Không đặt `active=True`, vì sẽ cập nhật bằng callback)
taskbar_vertical = dbc.Card(
    [
        dbc.CardHeader("Chọn Subpage"),
        dbc.Nav(
            [
                dbc.NavLink("Subpage 1", href="/page-1/subpage-1", id="subpage-1-link"),
                dbc.NavLink("Subpage 2", href="/page-1/subpage-2", id="subpage-2-link"),
                dbc.NavLink("Subpage 3", href="/page-1/subpage-3", id="subpage-3-link"),
                dbc.NavLink("Subpage 4", href="/page-1/subpage-4", id="subpage-4-link"),
            ],
            vertical=True,
            pills=True,
        ),
    ], 
    style={"width": "200px", "position": "relative", "height": "100vh", "padding": "10px"}
)

# Layout chính của Page 1
layout = dbc.Container([
    dcc.Location(id="url", refresh=False),  # Để theo dõi pathname
    dbc.Row([
        dbc.Col(taskbar_vertical, width=2),
        dbc.Col(html.Div(id="subpage-content"), width=10),
    ], style={"margin-top": "20px"})
], fluid=True)

# Callback cập nhật nội dung của subpage
@app.callback(
    Output("subpage-content", "children"),
    Input("url", "pathname")
)
def update_subpage(pathname):
    if pathname == "/page-1/subpage-1":
        return subpage_1.layout
    elif pathname == "/page-1/subpage-2":
        return subpage_2.layout
    elif pathname == "/page-1/subpage-3":
        return subpage_3.layout
    elif pathname == "/page-1/subpage-4":
        return subpage_4.layout
    else:
        return subpage_1.layout  

# Callback đổi màu NavLink khi subpage được chọn
@app.callback(
    [Output("subpage-1-link", "active"),
     Output("subpage-2-link", "active"),
     Output("subpage-3-link", "active"),
     Output("subpage-4-link", "active")],
    Input("url", "pathname"),
)
def highlight_active_subpage(pathname):
    return [pathname == "/page-1/subpage-1",
            pathname == "/page-1/subpage-2",
            pathname == "/page-1/subpage-3",
            pathname == "/page-1/subpage-4"]
