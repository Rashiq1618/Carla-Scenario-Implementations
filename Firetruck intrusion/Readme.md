This project simulates an autonomous vehicle navigating through a pre-defined route in CARLA Simulator while a firetruck intersects the path at a specific waypoint to test collision or avoidance behavior.

Spawns a Mini vehicle and a Firetruck in the CARLA world.

Automatically plans the longest route for the Mini using GlobalRoutePlanner.

Displays:

Live RGB camera feed from a drone-like top-rear view.

Speed, steering angle, and waypoint index overlaid on the camera view.

Simulates firetruck intrusion when the Mini reaches a specific waypoint.

Live control using throttle and steering angle toward next waypoints.

Requirements
CARLA Simulator (v0.9.15 recommended)

Python 3.7+

Installed Python dependencies

Running the Script
Start CARLA Simulator (CarlaUE4.exe)

Run the Python script:
python firetruck_intrusion_simulation.py
Press q in the OpenCV window to quit early.
