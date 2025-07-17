# Waypoint Follower in CARLA Simulator

This Python script enables a Tesla Model 3 vehicle to follow a series of waypoints autonomously within the CARLA simulator. It includes real-time visualization using Pygame and displays the vehicle's camera feed along with current speed.

## Features

- Vehicle automatically follows a predefined path using waypoint navigation
- Pure pursuit-style steering control
- Speed control with basic throttle and brake logic
- RGB camera sensor visualization in Pygame window
- Displays real-time vehicle speed

## Requirements

- [CARLA Simulator](https://carla.org/) (version 0.9.x)
- Python 3.7+
- `pygame`, `numpy`, `carla` Python API

## Installation

1. Clone this repository or copy the script into a `.py` file (e.g., `waypoint_follower.py`)
2. Install required Python packages:
   ```bash
   pip install pygame numpy
````

3. Ensure CARLA is running:

   ```bash
   ./CarlaUE4.sh
   ```

4. Run the script:

   ```bash
   python waypoint_follower.py
   ```

## How It Works

* The vehicle spawns at a random location and generates 120 forward waypoints.
* It uses a simple steering algorithm to align the heading with the next waypoint.
* Speed is regulated to a target of 25 km/h using throttle and brake.
* A rear-facing RGB camera shows live feed via a Pygame display.

## Notes

* Script assumes CARLA is running on `localhost:2000`
* All vehicles are removed before spawning a new one to avoid collisions
* This is a minimal example and does not include advanced planning or obstacle avoidance

## License

This project is open source under the MIT license.
