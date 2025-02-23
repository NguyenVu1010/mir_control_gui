from dash.dependencies import Input, Output
from dash import ctx  # Để lấy ID của nút được bấm
from .subpages import subpages  # Import subpages

def register_callbacks(app):
    @app.callback(
        Output("subpage1_content", "children"), 
        [Input("btn_subpage_1", "n_clicks"),
         Input("btn_subpage_2", "n_clicks"),
         Input("btn_subpage_3", "n_clicks"),
         Input("btn_subpage_4", "n_clicks")]
    )
    def update_subpage(n1, n2, n3, n4):
        button_id = ctx.triggered_id  # Lấy ID của nút vừa bấm

        if button_id == "btn_subpage_1":
            return subpages["subpage_1"]
        elif button_id == "btn_subpage_2":
            return subpages["subpage_2"]
        elif button_id == "btn_subpage_3":
            return subpages["subpage_3"]
        elif button_id == "btn_subpage_4":
            return subpages["subpage_4"]
        return subpages["subpage_1"]  # Mặc định hiển thị Subpage 1
