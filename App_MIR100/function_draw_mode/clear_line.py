# clear_line.py

from dash import Input, Output, State

def create_clear_line_callbacks(app):
    """
    Tạo callback để xóa các đường line đã vẽ từ dữ liệu lưu trữ 'saved-lines'.
    """

    @app.callback(
        Output('saved-lines', 'data'),
        Input('delete-line-button', 'n_clicks'),
        State('saved-lines', 'data'),
        State('map-image-draw-mode', 'figure'),
        prevent_initial_call=True
    )
    def delete_last_saved_line(n_clicks, saved_lines, figure):
        """
        Callback function to delete the last saved line.

        Args:
            n_clicks (int): The number of times the 'Delete Line' button has been clicked.
            saved_lines (list): A list of saved line coordinates.
            figure (dict): The current state of the map figure.

        Returns:
            list: Updated list of saved lines with the last one removed.
        """
        if n_clicks > 0 and saved_lines:
            saved_lines = saved_lines[:-1]
            if figure and 'layout' in figure:
              figure['layout']['shapes'] = figure['layout']['shapes'][:-1]
            return saved_lines
        else:
            return saved_lines