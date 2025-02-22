import dash
from dash import dcc, html, Input, Output
from pages import get_page  # Import hàm lấy trang từ pages

app = dash.Dash(__name__, suppress_callback_exceptions=True)

app.layout = html.Div([
    dcc.Tabs(id="tabs", value="tab-1", children=[
        dcc.Tab(label="Trang 1", value="tab-1"),
        dcc.Tab(label="Trang 2", value="tab-2"),
        dcc.Tab(label="Trang 3", value="tab-3"),
    ]),
    html.Div(id="content")
])

@app.callback(
    Output("content", "children"),
    Input("tabs", "value")
)
def update_tab(tab_name):
    return get_page(tab_name)

if __name__ == "__main__":
    app.run_server(debug=True)
