from dash import dcc, html
import dash_bootstrap_components as dbc
from utils.data import LANGUAGES

class StatusBar:
    def create_status_bar(self):
        return html.Div(
            [
                dbc.Row(
                    [
                        dbc.Col(
                            [
                                html.Span("No missions in queue", className="badge bg-warning text-dark me-2"),
                                html.Span("PAUSED", className="badge bg-danger me-2"),
                                html.Span("ALL OK", className="badge bg-success me-2"),
                            ],
                            width="auto",
                        ),
                        dbc.Col(
                            dcc.Dropdown(
                                id="language-dropdown",
                                options=LANGUAGES,
                                value="en",  # Default language
                                clearable=False,
                            ),
                            width="auto",
                            className="me-2"  # Add spacing before the joystick button
                        ),
                        dbc.Col(
                            dbc.Button("Open Joystick", id="open-joystick-btn", color="primary", size="sm"),
                            width="auto",
                            className="text-end",  # Right align the button
                        ),
                        html.Div(id="joystick-popup-container"),  # Container for Joystick popup
                    ],
                    align="center",  # Align items vertically
                    className="g-0",  # Remove default gaps between columns
                    style={"width": "100%"}, # Ensure the row takes up full width
                )
            ],
            style={
                "background": "#34495E",
                "color": "white",
                "padding": "10px",
                "position": "fixed",
                "top": 0,
                "left": "250px",
                "width": "100%",
                "zIndex": 1000,
            },
        )