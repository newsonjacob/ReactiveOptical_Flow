# UAV Optical Flow Navigation System

This project implements a real-time optical flow-based navigation system for an unmanned aerial vehicle (UAV) in the AirSim simulation environment. The UAV uses computer vision to detect motion in its environment and perform obstacle avoidance by braking, dodging, or nudging forward based on dynamic scene analysis.

## Features

* 🧠 Optical flow tracking using Lucas-Kanade method with CLAHE enhancement
* 📊 Flow history smoothing for left, center, and right partitions
* ✈️ Autonomous decision logic: brake, dodge, resume, and blind forward
* 🪟 GUI button to end the simulation safely
* 📁 Structured modular code with reusable components

## Project Structure

```
AirSimExperiments/
├── main.py               # Entry point of the program
├── uav/
│   ├── __init__.py       # Makes the uav folder a module
│   ├── perception.py     # Optical flow tracker and flow history
│   ├── navigation.py     # Obstacle avoidance and motion logic
│   └── interface.py      # GUI stop button
├── flow_logs/            # Output directory for log files
└── README.md             # You're here!
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
* OpenCV (`pip install opencv-python`)
* NumPy (`pip install numpy`)

## Running Tests

The unit tests use [pytest](https://pytest.org/). After installing the
requirements, run the tests from the project root:

```bash
pytest
```

## Running the Simulation

1. Launch the AirSim Unreal environment.
2. Run the script:

   ```bash
   python main.py
   ```
3. Click the GUI stop button to end the simulation cleanly.

## Example Log Format

```
frame,time,features,flow_left,flow_center,flow_right
42,13.23,117,4.320,10.827,6.910
```

## Future Improvements

* Add SLAM integration
* Visualization of flow vectors in real time
* Command line argument parsing for tuning thresholds
* ROS bridge for physical deployment
