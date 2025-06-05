# UAV Optical Flow Navigation System

This project implements a real-time optical flow-based navigation system for an unmanned aerial vehicle (UAV) in the AirSim simulation environment. The UAV uses computer vision to detect motion in its environment and perform obstacle avoidance by braking, dodging, or nudging forward based on dynamic scene analysis.

## Features

* 🧠 Optical flow tracking using Lucas-Kanade method with CLAHE enhancement
* 📊 Flow history smoothing for left, center, and right partitions
* ✈️ Autonomous decision logic: brake, dodge, resume, and blind forward
* 🪟 Simple GUI showing a STOP button, live flow magnitudes, and the current state
* 📁 Structured modular code with reusable components

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
2. **Tracking**: Optical flow tracks features between consecutive grayscale images.
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

3. The GUI displays the STOP button along with left/center/right flow values and the current state. Click STOP to end the simulation cleanly.

## Batch Runs

Execute multiple runs back to back using `batch_runs.py`:

```bash
python batch_runs.py --count 3
```

Omit `--count` to use the default of 5 runs.

## Visualizing Flight Logs

The `analysis/visualize_flight.py` script converts a CSV log and a set of
obstacles into an interactive HTML view. You can optionally filter which
obstacles are drawn:

```bash
python analysis/visualize_flight.py --log flow_logs/full_log_0001.csv \
    --obstacles analysis/obstacles.json --output flight.html \
    --show-ucx --show-huge
```

Flags:

* `--show-ucx` – include objects whose names start with `UCX_`.
* `--show-huge` – include very large bounding boxes.
* `--show-flat` – include large thin slices such as floors or walls.

## Example Log Format

```
frame,time,features,flow_left,flow_center,flow_right,flow_std,pos_x,pos_y,pos_z,yaw,speed,state,collided,brake_thres,dodge_thres,probe_req,fps
1,0.05,120,3.2,1.1,2.0,0.8,0.12,0.00,-2.00,0.0,1.7,resume,0,50.0,8.0,20.0,18.5
```

## Future Improvements

* Add SLAM integration
* Visualization of flow vectors in real time
* Command line argument parsing for tuning thresholds
* ROS bridge for physical deployment

## License

This project is licensed under the [MIT License](LICENSE).
