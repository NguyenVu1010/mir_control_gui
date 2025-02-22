from dash import html

layout = html.Div([
    html.H1("Trang 2 - Hiển thị thông tin"),
    html.P("Dữ liệu cảm biến của robot sẽ hiển thị tại đây."),
    html.Img(src="images/im1.png", style={"width": "100%"})
])
