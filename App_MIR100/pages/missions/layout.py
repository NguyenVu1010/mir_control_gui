from dash import html, dcc
import dash_ag_grid as dag
import dash_bootstrap_components as dbc

# Dữ liệu mẫu
columns = [
    {"field": "ID", "headerName": "ID", "width": 70},
    {"field": "Mission", "headerName": "Mission", "width": 150},
    {"field": "Status", "headerName": "Status", "width": 120},
]

data = [
    {"ID": 1, "Mission": "mission1", "Status": "In Progress"},
    {"ID": 2, "Mission": "mission2", "Status": "Pending"},
]

# Layout của bảng
def mission_queue_layout():
    return dbc.Container(
        [
            html.H4("Mission Queue", className="text-primary", style={"textAlign": "left"}),
            dag.AgGrid(
                id="mission-queue-table",
                columnDefs=columns,
                rowData=data,
                defaultColDef={"sortable": True, "filter": True, "resizable": True},
                style={"height": "300px", "width": "400px"},
                className="ag-theme-balham",
            ),
        ],
        style={"background": "#f8f9fa", "borderRadius": "10px", "padding": "15px", "boxShadow": "2px 2px 10px rgba(0,0,0,0.1)"},
    )
