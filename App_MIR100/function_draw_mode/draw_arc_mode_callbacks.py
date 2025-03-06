# draw_arc_mode_callbacks.py
from dash import Input, Output, State, callback, callback_context, no_update
import plotly.graph_objects as go
import numpy as np
import json
import math

def circle_from_3_points(P1, P2, P3):
    x1, y1 = P1
    x2, y2 = P2
    x3, y3 = P3
    A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2
    B = (x1**2 + y1**2) * (y3 - y2) + (x2**2 + y2**2) * (y1 - y3) + (x3**2 + y3**2) * (y2 - y1)
    C = (x1**2 + y1**2) * (x2 - x3) + (x2**2 + y2**2) * (x3 - x1) + (x3**2 + y3**2) * (x1 - x2)
    D = (x1**2 + y1**2) * (x3 * y2 - x2 * y3) + (x2**2 + y2**2) * (x1 * y3 - x3 * y1) + (x3**2 + y3**2) * (x2 * y1 - x1 * y2)
    if A == 0:
        return None  
    center_x = -B / (2 * A)
    center_y = -C / (2 * A)
    radius = np.sqrt((B**2 + C**2 - 4 * A * D) / (4 * A**2))
    return center_x, center_y, radius

def draw_arc(center_x, center_y, start_angle, end_angle, radius, color="red"):
    n_points = 50
    angles = np.linspace(start_angle, end_angle, n_points)
    x = center_x + radius * np.cos(angles)
    y = center_y + radius * np.sin(angles)
    return go.Scatter(
        x=x,
        y=y,
        mode="lines",
        line=dict(color=color, width=2),
        showlegend=False,
    )

# Callback to open the draw arc method modal
@callback(
    Output("draw-arc-method-modal", "is_open"),
    Input("draw-arc-button", "n_clicks"),
    State("draw-arc-mode", "data"),
    State("draw-arc-method-modal", "is_open"),
    prevent_initial_call=True,
)
def open_draw_arc_method_modal(n_clicks, draw_arc_mode, is_open):
    ctx = callback_context
    if not ctx.triggered:
        return no_update

    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == 'draw-arc-button' and draw_arc_mode:
        return False  
    return not is_open


# Callback to set the draw method (manual or coordinate) for arc and close the modal
@callback(
    Output("draw-arc-method", "data"),
    Output("draw-arc-method-modal", "is_open", allow_duplicate=True),
    Input("manual-draw-arc-button", "n_clicks"),
    Input("coordinate-draw-arc-button", "n_clicks"),
    State("draw-arc-method-modal", "is_open"),
    prevent_initial_call=True,
)
def set_draw_arc_method(manual_clicks, coordinate_clicks, is_open):
    ctx = callback_context
    if not ctx.triggered:
        return no_update, no_update

    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == "manual-draw-arc-button":
        return "manual", False
    elif button_id == "coordinate-draw-arc-button":
        return "coordinate", False
    return "", is_open

# Callback to open coordinate modal for arc when "coordinate" draw method is selected
@callback(
    Output("coordinate-arc-modal", "is_open"),
    Input("draw-arc-method", "data"),
    State("coordinate-arc-modal", "is_open"),
    prevent_initial_call=True,
)
def open_coordinate_arc_modal(draw_arc_method, is_open):
    if draw_arc_method == "coordinate":
        return True
    return False


# Callback to draw arc based on coordinates entered in the modal
@callback(
    Output("map-image-draw-mode", "figure", allow_duplicate=True),
    Input("draw-arc-button-coordinate", "n_clicks"),
    State("point1-x", "value"),
    State("point1-y", "value"),
    State("point2-x", "value"),
    State("point2-y", "value"),
    State("point3-x", "value"),
    State("point3-y", "value"),
    State("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def draw_arc_coordinate(n_clicks, p1x, p1y, p2x, p2y, p3x, p3y, figure):
    if n_clicks is None:
        return no_update
    try:
        p1 = (float(p1x), float(p1y))
        p2 = (float(p2x), float(p2y))
        p3 = (float(p3x), float(p3y))
    except (ValueError, TypeError):
        print("Invalid coordinates entered for arc.")
        return figure
    circle_params = circle_from_3_points(p1, p2, p3)
    if circle_params is None:
        print("Could not determine circle from these points.")
        return figure
    center_x, center_y, radius = circle_params
    start_angle = math.atan2(p1[1] - center_y, p1[0] - center_x)
    end_angle = math.atan2(p3[1] - center_y, p3[0] - center_x)
    arc = draw_arc(center_x, center_y, start_angle, end_angle, radius)
    figure["data"].append(arc)
    return figure


# Callback to store the start point when the user clicks on the graph for arc (manual mode)
@callback(
    Output("arc-coordinates", "data"),
    Input("map-image-draw-mode", "clickData"),
    State("draw-arc-method", "data"),
     State("draw-arc-mode", "data"), 

    prevent_initial_call=True,
)
def store_start_point_arc(clickData, draw_arc_method, draw_arc_mode):
    # print("store_start_point_arc called")
    # print("draw_arc_method:", draw_arc_method)
    # print("clickData:", clickData)
    if draw_arc_method == "manual" and clickData and draw_arc_mode: 
        x = clickData["points"][0]["x"]
        y = clickData["points"][0]["y"]
        return {"center_x": x, "center_y": y}
    return {}

# Callback to draw the arc when the user releases the mouse (relayoutData) for manual mode
@callback(
    Output("map-image-draw-mode", "figure", allow_duplicate=True),
    Input("map-image-draw-mode", "relayoutData"),
    Input("draw-arc-method", "data"),
    State("arc-coordinates", "data"),
    State("map-image-draw-mode", "figure"),
     State("draw-arc-mode", "data"),
    prevent_initial_call=True,
)
def draw_arc_on_release(relayoutData, draw_arc_method, arc_coordinates, figure, draw_arc_mode):
    #  print("draw_arc_on_release called")
    #  print("draw_arc_method:", draw_arc_method)
    #  print("relayoutData:", json.dumps(relayoutData))  # Print relayoutData
    #  print("arc_coordinates:", arc_coordinates) # Add this line
    #  print("draw_arc_mode:", draw_arc_mode)  
     if draw_arc_method == "manual" and relayoutData and arc_coordinates and "center_x" in arc_coordinates and draw_arc_mode: 
        if 'xaxis.range[0]' in relayoutData and 'yaxis.range[0]' in relayoutData and 'xaxis.range[1]' in relayoutData and 'yaxis.range[1]' in relayoutData:
            center_x = arc_coordinates["center_x"]
            center_y = arc_coordinates["center_y"]
            end_x = relayoutData['xaxis.range[1]']
            end_y = relayoutData['yaxis.range[0]']
            radius = np.sqrt((end_x - center_x)**2 + (end_y - center_y)**2)
            start_angle = 0  
            end_angle = 2 * np.pi 
            arc = draw_arc(center_x, center_y, start_angle, end_angle, radius, color="purple")  

            figure["data"].append(arc)
            return figure
        else:
            return no_update  
     else:
        return no_update


# Callback to clear the stored start point after drawing arc
@callback(
    Output("arc-coordinates", "data", allow_duplicate=True),
    Input("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def clear_start_point_arc(figure):
    return {}

@callback(
    Output("draw-arc-mode", "data"),
    Input("draw-arc-button", "n_clicks"),
    State("draw-arc-mode", "data"),
    prevent_initial_call=True,
)
def toggle_draw_arc_mode(n_clicks, current_state):
    return not current_state

@callback(
    Output("map-image-draw-mode", "dragmode", allow_duplicate=True),
    Input("draw-line-mode", "data"),
    Input("draw-arc-mode", "data"),
    prevent_initial_call=True,
)
def update_drag_mode(draw_line_mode, draw_arc_mode):
    ctx = callback_context
    triggered_id = ctx.triggered[0]['prop_id'].split('.')[0] if ctx.triggered else None

    if triggered_id == "draw-line-mode":
        if draw_line_mode:
            return "drawline" 
        else:
            return "pan"
    elif triggered_id == "draw-arc-mode":
        if draw_arc_mode:
            return "drawarc" 
        else:
            return "pan"
    
    return "pan" 

@callback(
    Output("draw-arc-button", "style"),
    Input("draw-arc-mode", "data"),
     State("button-style-store", "data"),
    prevent_initial_call=True
)
def update_button_arc_style(is_active, button_style_store):
    default_style = button_style_store["draw_arc_button"]
    active_button_style = {
        "padding": "8px 16px",
        "border": "1px solid #2ecc71",
        "color": "white",
        "background-color": "#2ecc71",
        "border-radius": "5px",
        "transition": "all 0.3s ease-in-out",
        "cursor": "pointer",
    }
    if is_active:
        return active_button_style
    else:
        return default_style