This project demonstrates a distributed pedestrian detection and vehicle control system using the CARLA simulator across two computers. walker_detector.py runs the simulation, spawns an autonomous vehicle, and detects nearby pedestrians using vector math based on position and orientation. controller.py receives the detection data and makes braking decisions, sending commands back to control the ego vehicle remotely. Communication between the two systems is handled via UDP sockets.

Key features:

Real-time pedestrian detection using direction vectors and dot product logic

Autonomous braking and resume logic based on proximity and angle

UDP-based communication between simulation and control modules

Simple RGB camera visualization for simulation context

To run the system, execute walker_detector.py on the CARLA simulation machine and controller.py on the other system. Make sure both machines are on the same network and IPs/ports are correctly configured.
