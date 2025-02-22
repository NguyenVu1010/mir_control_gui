from dash import html
import dash_bootstrap_components as dbc

# Taskbar ngang cố định
taskbar_horizontal = dbc.NavbarSimple(
    children=[
        dbc.NavItem(dbc.NavLink("Page 1", href="/page-1", className="nav-link")),
        dbc.NavItem(dbc.NavLink("Page 2", href="/page-2", className="nav-link")),
        dbc.NavItem(dbc.NavLink("Page 3", href="/page-3", className="nav-link")),
    ],
    brand="MiR Control",
    brand_href="/page-1",
    color="primary",
    dark=True,
    className="taskbar-horizontal",
)

# Taskbar dọc theo từng trang
def get_taskbar_vertical(page):
    """Tạo taskbar dọc theo từng page."""
    if page == "page-1":
        return dbc.Card([
            dbc.CardHeader("Chọn Subpage"),
            dbc.Nav([
                dbc.NavLink("Subpage 1", href="/page-1/subpage-1", className="sub-nav"),
                dbc.NavLink("Subpage 2", href="/page-1/subpage-2", className="sub-nav"),
                dbc.NavLink("Subpage 3", href="/page-1/subpage-3", className="sub-nav"),
                dbc.NavLink("Subpage 4", href="/page-1/subpage-4", className="sub-nav"),
            ], vertical=True, pills=True, className="taskbar-vertical")
        ])
    elif page == "page-2":
        return dbc.Card([
            dbc.CardHeader("Tùy chọn Page 2"),
            dbc.Nav([
                dbc.NavLink("Option 1", href="/page-2/option-1", className="sub-nav"),
                dbc.NavLink("Option 2", href="/page-2/option-2", className="sub-nav"),
            ], vertical=True, pills=True, className="taskbar-vertical")
        ])
    else:
        return None  # Không có taskbar dọc
