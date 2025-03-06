# draw_arc_callback.py
from dash import Output, Input, State, callback, callback_context, no_update
import plotly.graph_objects as go
import json

# Callback to store arc's start, middle, and end point when the user clicks on the graph

@callback(
    Output("arc-coordinates", "data"),
    Input("map-image-draw-mode", "clickData"),
    State("arc-draw-method", "data"),
    State("draw-arc-mode", "data"),
    State("arc-coordinates", "data"),

    prevent_initial_call=True,
)
def store_arc_points(clickData, draw_method, draw_arc_mode, arc_coordinates):
    if draw_method == "manual" and clickData and draw_arc_mode:
        x = clickData["points"][0]["x"]
        y = clickData["points"][0]["y"]
        print("Manual Drawing the Arc.")
        print(arc_coordinates)

        if 'points' not in arc_coordinates:
            arc_coordinates['points'] = []

        arc_coordinates['points'].append((x, y))


        if len(arc_coordinates['points']) <= 3:
                return arc_coordinates
        else:
            return {'points':[(x, y)]}
    else:
        print("No Click Detected on drawing mode arc")
        return arc_coordinates


# Callback to draw arc based on collected points when the user release the click.
@callback(
    Output("map-image-draw-mode", "figure", allow_duplicate=True),
    Input("arc-coordinates", "data"),
    State("map-image-draw-mode", "figure"),
    State("draw-arc-mode", "data"),
    State("arc-draw-method", "data"),

    prevent_initial_call=True,
)
def draw_arc_manual(arc_coordinates, figure, draw_arc_mode, draw_method):

    if draw_method == "manual" and draw_arc_mode and arc_coordinates:
        print("Try Drawing Manual Arc")
        if 'points' in arc_coordinates and len(arc_coordinates['points']) == 3:

            print("Manual Drawing 3 points for arc: " )
            start_x, start_y = arc_coordinates['points'][0]
            middle_x, middle_y = arc_coordinates['points'][1]
            end_x, end_y = arc_coordinates['points'][2]

            # Draw arc to figure's layout
            arc_shape = go.layout.Shape(
                type="path",
                path=f"M {start_x},{start_y} Q {middle_x},{middle_y} {end_x},{end_y}",
                line=dict(color="red", width=2),
            )

            figure['layout']['shapes'].append(arc_shape)

            #Clear coordinate list

            return figure
        else:
             return figure

    else:
        print("Arc Drawing Faild in some where !")
        return figure


@callback(
    Output("arc-coordinates", "data", allow_duplicate=True),
    Input("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def clear_arc_points(figure):
        print("clear points.")
        return {'points':[]}