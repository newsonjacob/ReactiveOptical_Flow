import os
import fnmatch


def retain_recent_files(dir_path: str, pattern: str, keep: int = 5) -> None:
    """Keep only the ``keep`` most recent files matching ``pattern``.

    Parameters
    ----------
    dir_path : str
        Directory to search for files.
    pattern : str
        Glob pattern used to select files within ``dir_path``.
    keep : int, optional
        Number of recent files to preserve. Older files are removed.
    """
    try:
        files = [
            os.path.join(dir_path, f)
            for f in os.listdir(dir_path)
            if fnmatch.fnmatch(f, pattern)
        ]
    except FileNotFoundError:
        return

    files.sort(key=os.path.getmtime, reverse=True)

    for old_file in files[keep:]:
        try:
            os.remove(old_file)
        except OSError:
            pass


def retain_recent_views(view_dir: str, keep: int = 5) -> None:
    """Keep only the ``keep`` most recent ``flight_view_*.html`` files."""

    retain_recent_files(view_dir, "flight_view_*.html", keep)
