from dash import html, dcc  # Thêm dcc vào import

layout = html.Div([
    html.H1("Trang 3 - Cấu hình"),
    html.P("Thiết lập các thông số của robot tại đây."),
    dcc.Input(placeholder="Nhập thông số...", type="text"), 
    html.Button("Lưu", id="save-btn", n_clicks=0)
])
