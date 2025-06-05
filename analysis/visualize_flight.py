import pandas as pd
import json
import numpy as np
import argparse
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as R


def load_telemetry(log_path):
    df = pd.read_csv(log_path)
    x_col, y_col, z_col = 'pos_x', 'pos_y', 'pos_z'
    return df[[x_col, y_col, z_col]].values, x_col, y_col, z_col


def load_obstacles(obstacle_path):
    with open(obstacle_path, 'r') as f:
        return json.load(f)


def find_alignment_marker(obstacles, marker_name="PlayerStart_3"):
    for obj in obstacles:
        if obj['name'] == marker_name:
            return np.array(obj['location'])
    raise ValueError(f"Marker '{marker_name}' not found in obstacle data.")


def compute_offset(uav_start, marker_position, scale=1.0):
    return marker_position - uav_start * scale


def draw_box(center, dimensions, rotation_euler):
    cx, cy, cz = center
    dx, dy, dz = [d / 2.0 for d in dimensions]
    corners = np.array([
        [-dx, -dy, -dz], [ dx, -dy, -dz], [ dx,  dy, -dz], [-dx,  dy, -dz],
        [-dx, -dy,  dz], [ dx, -dy,  dz], [ dx,  dy,  dz], [-dx,  dy,  dz],
    ])
    rot = R.from_euler('xyz', rotation_euler).as_matrix()
    rotated = np.dot(corners, rot.T) + np.array(center)
    edges = [(0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4),
             (0,4), (1,5), (2,6), (3,7)]
    lines = []
    for start, end in edges:
        xs, ys, zs = zip(rotated[start], rotated[end])
        lines.append(go.Scatter3d(x=xs, y=ys, z=zs, mode='lines',
                                  line=dict(color='red', width=2), showlegend=False))
    return lines


def build_plot(uav_path, obstacles, offset, scale,
               show_ucx=False, show_huge=False, show_flat=False):
    uav_scaled = uav_path * scale
    uav_flipped = np.column_stack((uav_scaled[:, 0], -uav_scaled[:, 1], uav_scaled[:, 2]))
    uav_aligned = uav_flipped + offset

    path_trace = go.Scatter3d(
        x=uav_aligned[:, 0],
        y=uav_aligned[:, 1],
        z=uav_aligned[:, 2],
        mode='lines',
        line=dict(color='blue', width=4),
        name='UAV Path'
    )

    box_lines = []
    for obs in obstacles:
        name = obs['name']
        loc = np.array(obs['location'])
        dims = np.array(obs['dimensions'])

        if not show_ucx and name.startswith("UCX_"):
            continue
        if np.linalg.norm(loc) < 1.0:  # skip objects at origin
            continue
        if not show_huge and np.max(dims) > 1000:
            continue
        if not show_flat and np.any(dims > 200) and dims[1] > 100:
            continue

        box_lines.extend(draw_box(obs['location'], obs['dimensions'], obs['rotation']))

    fig = go.Figure(data=[path_trace] + box_lines)
    fig.update_layout(
        scene=dict(xaxis_title='X', yaxis_title='Y', zaxis_title='Z'),
        title="UAV Path vs Obstacle Course",
        showlegend=True
    )
    return fig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', required=True, help="Path to UAV flight log CSV")
    parser.add_argument('--obstacles', default="analysis/obstacles.json", help="Path to obstacles JSON")
    parser.add_argument('--output', default="uav_debug_view.html", help="Output HTML file")
    parser.add_argument('--scale', type=float, default=1.0, help="Scale multiplier for UAV data")
    parser.add_argument('--show-ucx', action='store_true', help="Include obstacles with names starting with 'UCX_'")
    parser.add_argument('--show-huge', action='store_true', help="Include very large obstacles")
    parser.add_argument('--show-flat', action='store_true', help="Include large thin obstacles like floors or walls")
    args = parser.parse_args()

    telemetry, *_ = load_telemetry(args.log)
    obstacles = load_obstacles(args.obstacles)
    marker = find_alignment_marker(obstacles)
    offset = compute_offset(telemetry[0], marker, scale=args.scale)

    fig = build_plot(
        telemetry,
        obstacles,
        offset,
        scale=args.scale,
        show_ucx=args.show_ucx,
        show_huge=args.show_huge,
        show_flat=args.show_flat,
    )
    fig.write_html(args.output)
    print(f"✅ Visualization saved to {args.output}")


if __name__ == '__main__':
    main()
