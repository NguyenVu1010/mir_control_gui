from dash import Dash, html, dcc
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output


app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP], suppress_callback_exceptions=True)
app.title = "MiR Control"
from pages import page_control, page_2, page_3
# Thanh Taskbar ngang cố định
navbar = dbc.NavbarSimple(
    children=[
        dbc.NavItem(dbc.NavLink("Page 1", href="/page-1")),
        dbc.NavItem(dbc.NavLink("Page 2", href="/page-2")),
        dbc.NavItem(dbc.NavLink("Page 3", href="/page-3")),
    ],
    brand="MiR Control",
    brand_href="/",
    color="primary",
    dark=True,  
    style={"width": "100%", "padding": "15px", "font-size": "20px"}
)

app.layout = dbc.Container([
    dcc.Location(id="url", refresh=False),
    navbar,
    html.Div(id="page-content", style={"margin-top": "20px"})
], fluid=True)

@app.callback(
    Output("page-content", "children"),
    Input("url", "pathname")
)
def display_page(pathname):
    if pathname in ["/", "/page-1"]:  
        return page_control.layout
    elif pathname == "/page-2":
        return page_2.layout
    elif pathname == "/page-3":
        return page_3.layout
    else:
        return html.H3("404 - Không tìm thấy trang!")

if __name__ == "__main__":
    app.run_server(debug=True)
