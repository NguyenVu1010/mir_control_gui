# draw_mode.py
from dash import html, dcc
import dash_bootstrap_components as dbc
import plotly.graph_objects as go

def create_draw_mode_layout():
    button_style = {
        "padding": "8px 16px",
        "border": "1px solid #3498db",
        "color": "#3498db",
        "background-color": "white",
        "border-radius": "5px",
        "transition": "all 0.3s ease-in-out",
        "cursor": "pointer",
    }

    button_hover_style = {
        "background-color": "#3498db",
        "color": "white",
    }

    # Tạo grid và hệ tọa độ
    grid_size = 50  # Kích thước mỗi ô grid
    grid_color = "#CCCCCC"  # Màu grid
    axis_color = "#000000"  # Màu trục tọa độ

    # Tạo các đường grid
    grid_lines = []
    for i in range(0, 1000, grid_size):
        grid_lines.append(
            go.Scatter(
                x=[i, i],
                y=[0, 1000],
                mode="lines",
                line=dict(color=grid_color, width=1),
                hoverinfo="none",
                showlegend=False,
            )
        )
        grid_lines.append(
            go.Scatter(
                x=[0, 1000],
                y=[i, i],
                mode="lines",
                line=dict(color=grid_color, width=1),
                hoverinfo="none",
                showlegend=False,
            )
        )

    # Tạo trục tọa độ
    axis_lines = [
        go.Scatter(
            x=[0, 1000],
            y=[0, 0],
            mode="lines",
            line=dict(color=axis_color, width=2),
            hoverinfo="none",
            showlegend=False,
        ),
        go.Scatter(
            x=[0, 0],
            y=[0, 1000],
            mode="lines",
            line=dict(color=axis_color, width=2),
            hoverinfo="none",
            showlegend=False,
        ),
    ]

    # Thêm hình ảnh bản đồ làm nền
    map_image = go.layout.Image(
        source="/static/map_image.png",  # Đường dẫn đến hình ảnh bản đồ
        xref="x",
        yref="y",
        x=0,
        y=1000,
        sizex=1000,
        sizey=1000,
        sizing="stretch",
        layer="below",  # Đặt hình ảnh ở lớp dưới cùng
    )

    # Tạo layout cho bản đồ
    map_layout = go.Layout(
        xaxis=dict(showgrid=False, range=[0, 1000]),
        yaxis=dict(showgrid=False, range=[0, 1000], scaleanchor="x", scaleratio=1),
        plot_bgcolor="white",
        margin=dict(l=0, r=0, t=0, b=0),
        images=[map_image],  # Thêm hình ảnh bản đồ vào layout
        shapes=[],  # Các hình vẽ sẽ được thêm vào đây
    )

    # Tạo figure với grid, trục tọa độ và hình ảnh bản đồ
    map_figure = go.Figure(data=grid_lines + axis_lines, layout=map_layout)

    draw_mode_layout = html.Div(
        [
            html.H3("Draw Mode", className="mb-3", style={"color": "#2C3E50"}),
            html.P("This is the draw mode page. Edit and draw the map.", className="text-muted"),
            dbc.ButtonGroup(
                [
                    dbc.Button(
                        [html.I(className="fas fa-pen-nib me-2"), "Draw Line"],
                        id="draw-line-button",
                        style=button_style,
                        className="me-1",
                    ),
                    dbc.Button(
                        [html.I(className="fas fa-vector-square me-2"), "Draw Arc"],
                        id="draw-arc-button",
                        style=button_style,
                    ),
                ],
                className="mb-3",
            ),
            dcc.Store(id='draw-line-mode', data=False),  # Store for draw line mode
            dcc.Store(id='line-coordinates', data=[]),  # Store for storing line coordinates
            dcc.Store(id='draw-method', data=''),  # Store drawing method: 'manual' or 'coordinate'
            dcc.Graph(
                id="map-image-draw-mode",
                figure=map_figure,
                style={"width": "100%", "height": "600px", "border": "2px solid #34495E"},
                config={"scrollZoom": True},  # Cho phép zoom bằng scroll
            ),
            html.P("The map is ready for your work.", className="text-info mt-2"),
            html.Div(id='draw-mode-output'),  # Output after drawing
            dbc.Modal(  # Modal for choosing draw method
                [
                    dbc.ModalHeader(dbc.ModalTitle("Choose Drawing Method")),
                    dbc.ModalBody(
                        [
                            dbc.Button("Manual Draw", id="manual-draw-button", color="primary", className="me-2"),
                            dbc.Button("Enter Coordinates", id="coordinate-draw-button", color="secondary"),
                        ]
                    ),
                ],
                id="draw-method-modal",
                is_open=False,
            ),
            dbc.Modal(  # Modal for coordinate input
                [
                    dbc.ModalHeader(dbc.ModalTitle("Enter Coordinates")),
                    dbc.ModalBody(
                        [
                            dbc.Label("Start X:"),
                            dbc.Input(type="number", id="start-x"),
                            dbc.Label("Start Y:"),
                            dbc.Input(type="number", id="start-y"),
                            dbc.Label("End X:"),
                            dbc.Input(type="number", id="end-x"),
                            dbc.Label("End Y:"),
                            dbc.Input(type="number", id="end-y"),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button("Draw", id="draw-button", color="primary"),
                            dbc.Button("Cancel", id="cancel-button"),
                        ]
                    ),
                ],
                id="coordinate-modal",
                is_open=False,
            ),
        ],
        style={
            "padding": "20px",
            "flex": "1",
            "background": "#ECF0F1",
            "marginLeft": "250px",
            "marginTop": "50px",
            "overflowY": "auto",
        },
    )
    return draw_mode_layout