import os


def retain_recent_views(view_dir: str, keep: int = 5) -> None:
    """Keep only the ``keep`` most recent ``flight_view_*.html`` files.

    Parameters
    ----------
    view_dir : str
        Directory containing the generated HTML files.
    keep : int, optional
        Number of recent files to preserve. Older files are removed.
    """
    try:
        views = [
            os.path.join(view_dir, f)
            for f in os.listdir(view_dir)
            if f.startswith("flight_view_") and f.endswith(".html")
        ]
    except FileNotFoundError:
        return

    views.sort(key=os.path.getmtime, reverse=True)

    for old_view in views[keep:]:
        try:
            os.remove(old_view)
        except OSError:
            pass
