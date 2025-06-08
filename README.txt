# UAV Optical Flow Navigation System

This project implements a real-time optical flow-based navigation system for an unmanned aerial vehicle (UAV) in the AirSim simulation environment. The UAV uses computer vision to detect motion in its environment and perform obstacle avoidance by braking, dodging, or nudging forward based on dynamic scene analysis.

## Features

* 🧠 Optical flow tracking uses CLAHE-enhanced frames for stable feature detection
* 📊 Flow history smoothing for left, center, and right partitions
* ✈️ Autonomous decision logic: brake, dodge, resume, and blind forward
* 🪟 Simple GUI showing a STOP button, live flow magnitudes, and the current state
* 📁 Structured modular code with reusable components
* 🚀 Perception tasks run asynchronously in a background thread using a dedicated
  `MultirotorClient` instance so navigation RPCs never clash with the main loop
* 🏁 Optional goal detection to land automatically when the UAV reaches the end of the course

## Project Structure

```
AirSimExperiments/
├── main.py               # Entry point of the program (calls main())
├── uav/
│   ├── __init__.py       # Makes the uav folder a module
│   ├── perception.py     # Optical flow tracker and flow history
│   ├── navigation.py     # Obstacle avoidance and motion logic
│   └── interface.py      # Tkinter GUI displaying flow data and STOP
├── flow_logs/            # Output directory for log files
└── README.txt             # You're here!
```

## How It Works

1. **Startup phase**: The UAV takes off and waits a few frames to stabilize.
2. **Tracking**: Each grayscale frame is contrast-enhanced with CLAHE before detecting features, then optical flow tracks them between frames.
3. **Analysis**: Magnitude of motion vectors is averaged across image partitions.
4. **Navigation decisions**:

   * Brake if the central region indicates rapid approaching flow.
   * Dodge if there's an obstacle in front but one side is safer.
   * Resume or reinforce forward motion otherwise.
   * Nudge if stuck with low flow and low speed.
5. **Logging**: Each frame’s data is saved to CSV for analysis.

## Requirements

* Python 3.8+
* AirSim installed and configured
* Python dependencies listed in `requirements.txt`
* Additional packages: `pandas`, `plotly`, and `scipy`

Install the Python packages with:

```bash
pip install -r requirements.txt
```

## Running Tests

The unit tests use [pytest](https://pytest.org/). After installing the
requirements, run the tests from the project root:

```bash
pytest
```

## Running the Simulation

1. Launch the AirSim Unreal environment.
2. Run the script. You can override the path to the Unreal Engine executable using `--ue4-path` or by setting the `UE4_PATH` environment variable:

   ```bash
   python main.py --ue4-path "C:\\Path\\To\\Blocks.exe"
   ```
   If omitted, the path defaults to `UE4_PATH` if set, otherwise to the value used during development.

3. After connecting, the program launches the Tkinter GUI using the controller's
   live parameter references. The window shows the STOP button together with
   left/center/right flow magnitudes and the current state. Click STOP to end
   the simulation cleanly.

## Batch Runs

Execute multiple runs back to back using `batch_runs.py`:

```bash
python batch_runs.py --count 3
```

Omit `--count` to use the default of 5 runs.

## Example Log Format

```
frame,time,features,flow_left,flow_center,flow_right,flow_std,pos_x,pos_y,pos_z,yaw,speed,state,collided,obstacle,side_safe,brake_thres,dodge_thres,probe_req,fps,simgetimage_s,decode_s,processing_s,loop_s
1,0.05,120,3.2,1.1,2.0,0.8,0.12,0.00,-2.00,0.0,1.7,resume,0,0,1,50.0,8.0,20.0,18.5,0.01,0.01,0.0,0.05
```

## Logging

Running `DroneController.run()` automatically creates a file like
`flow_logs/full_log_YYYYMMDD_HHMMSS.csv`. The first processed frame writes the
CSV header and each subsequent call to `log_frame()` appends a new row. Older
logs are cleaned up so only the most recent few are kept.

## Parameters

`FLOW_STD_MAX` controls the maximum tolerated variance of optical flow
magnitudes. When the standard deviation reported by the tracker exceeds
this threshold the probe-based fallback dodge logic is skipped because
the motion estimate is unreliable. The default value is `10.0`.

## Summarizing Runs

Gather quick statistics about each run with:

```bash
python analysis/summarize_runs.py
```

The script scans `flow_logs/full_log_*.csv` and prints the frame count,
how many frames registered a collision, and the straight‑line distance
from the first to the last recorded position.

## Flight Visualization

`main.py` automatically generates a `flight_view_*.html` file inside the
`analysis/` directory after each run. You can create or recreate the view
manually with:

```bash
python analysis/visualize_flight.py --log flow_logs/full_log_XXXX.csv --output analysis/flight_view_XXXX.html
```

Open the resulting HTML file in your browser to explore the flight path
interactively.

Only the five most recent `flight_view_*.html` files are retained in the
`analysis/` directory. Older visualizations are removed automatically.
If the log contains no telemetry, the visualization script now prints a
message and exits cleanly.

## Flight Review

For a combined summary and visualization refresh run:

```bash
python analysis/review_runs.py
```

This script iterates over `flow_logs/full_log_*.csv` files, generates any
missing `flight_view_*.html` pages and writes `analysis/summary_report.txt`
with basic statistics for each log.

## Future Improvements

* Add SLAM integration
* Visualization of flow vectors in real time
* Command line argument parsing for tuning thresholds
* ROS bridge for physical deployment

## Contributing

See `AGENTS.md` for developer guidelines. Always run `pytest` before submitting changes.

## License

This project is licensed under the [MIT License](LICENSE).
