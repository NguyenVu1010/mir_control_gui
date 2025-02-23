from dash import dcc, html
import dash_bootstrap_components as dbc

class MapSection:
    def create_map_section(self):
        return html.Div(
            [
                html.H3("Main Floor", className="mb-3", style={"color": "#2C3E50"}),
                html.P("Edit and draw the map", className="text-muted"),
                html.Img(src="/home/duc/Downloads/gui_map/map_image.png", style={"width": "100%", "border": "2px solid #34495E"}),
                html.P("The map is ready for your work.", className="text-info mt-2"),
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