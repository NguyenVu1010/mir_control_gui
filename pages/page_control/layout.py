from dash import html
import dash_bootstrap_components as dbc
from .subpages import subpage_1, subpage_2, subpage_3, subpage_4
from dash.dependencies import Input, Output
from main import app  

taskbar_vertical = dbc.Card(
    [
        dbc.CardHeader("Chọn Subpage"),
        dbc.Nav(
            [
                dbc.NavLink("Subpage 1", href="/page-1/subpage-1", active=True),
                dbc.NavLink("Subpage 2", href="/page-1/subpage-2"),
                dbc.NavLink("Subpage 3", href="/page-1/subpage-3"),
                dbc.NavLink("Subpage 4", href="/page-1/subpage-4"),
            ],
            vertical=True,
            pills=True,
        ),
    ], 
    style={"width": "200px", "position": "fixed", "height": "100vh", "padding": "10px"}
)

layout = dbc.Container([
    dbc.Row([
        dbc.Col(taskbar_vertical, width=2),
        dbc.Col(html.Div(id="subpage-content"), width=10),
    ], style={"margin-top": "20px"})
], fluid=True)

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
