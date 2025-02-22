from .page_1 import layout as layout1
from .page_2 import layout as layout2
from .page_3 import layout as layout3
from dash import html

# Dictionary lưu các trang
pages = {
    "tab-1": layout1,
    "tab-2": layout2,
    "tab-3": layout3
}

# Hàm lấy trang theo tab_name
def get_page(tab_name):
    return pages.get(tab_name, html.Div("Trang không tồn tại"))
