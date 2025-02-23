from dash import dcc, html
import dash_bootstrap_components as dbc

class Sidebar:
    def create_sidebar(self):
        return html.Div(
            [
                html.H2("GUI MIR100", className="text-white text-center"),
                html.Hr(className="text-white"),
                dbc.Nav(
                    [
                        dbc.NavLink("Missions", href="#", id="missions-link", className="text-white"),
                        dbc.NavLink("Maps", href="#", id="maps-link", className="text-white"),
                        dbc.NavLink("Sounds", href="#", id="sounds-link", className="text-white"),
                        dbc.NavLink("Transitions", href="#", id="transitions-link", className="text-white"),
                        dbc.NavLink("I/O Modules", href="#", id="io-modules-link", className="text-white"),
                        dbc.NavLink("Users", href="#", id="users-link", className="text-white"),
                        dbc.NavLink("User Groups", href="#", id="user-groups-link", className="text-white"),
                        dbc.NavLink("Paths", href="#", id="paths-link", className="text-white"),
                        dbc.NavLink("Path Guides", href="#", id="path-guides-link", className="text-white"),
                        dbc.NavLink("Marker Types", href="#", id="marker-types-link", className="text-white"),
                        dbc.NavLink("Footprints", href="#", id="footprints-link", className="text-white"),
                        dbc.NavLink("Change Password", href="#", id="change-password-link", className="text-white"),
                    ],
                    vertical=True,
                    pills=True,
                    id="sidebar-nav",
                ),
            ],
            style={
                "background-color": "#2C3E50",
                "padding": "20px",
                "width": "250px",
                "height": "100vh",
                "color": "white",
                "position": "fixed",
            },
        )