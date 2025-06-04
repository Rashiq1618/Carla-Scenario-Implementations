import carla
import numpy as np
import time

# Distance thresholds for braking and acceleration
STOP_DISTANCE = 10.0
FOLLOW_DISTANCE = 15.0  # Target distance to maintain from the lead vehicle
MAX_THROTTLE = 0.6  # Limit max acceleration
BRAKE_FORCE = 1.0  # Full braking Forcre

def lidar_callback(point_cloud, ego_vehicle, lead_vehicle):
    """  """

    # Get vehicle locations
    ego_location = ego_vehicle.get_transform().location
    lead_location = lead_vehicle.get_transform().location

    # Compute actual distance
    actual_distance = ego_location.distance(lead_location)
    print(f"Actual Distance: {actual_distance:.2f} meters")

    # Vehicle control logic
    control = ego_vehicle.get_control()

    if actual_distance < STOP_DISTANCE:
        print("Too close to lead vehicle! Braking...")
        control.throttle = 0.0
        control.brake = BRAKE_FORCE  # Apply full brakes

    elif actual_distance < FOLLOW_DISTANCE:
        print("Maintaining distance, slowing down")
        control.throttle = 0.3  # Slight acceleration
        control.brake = 0.0  # No braking

    else:
        print("Safe distance, accelerating.")
        control.throttle = MAX_THROTTLE  # Increase speed to catch up
        control.brake = 0.0  # Release brakes
    
    ego_vehicle.apply_control(control)

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    blueprint_library = world.get_blueprint_library()

    ego_vehicle_bp = blueprint_library.find("vehicle.audi.tt")  # Audi TT as ego vehicle
    lead_vehicle_bp = blueprint_library.find("vehicle.tesla.model3")  # Tesla Model 3 as lead vehicle

    spawn_points = world.get_map().get_spawn_points()
    if len(spawn_points) < 2:
        print("Not enough spawn points available!")
        return

    # Ensure the vehicles are spawned in the same lane, one behind the other
    lead_spawn = spawn_points[0]
    ego_spawn = carla.Transform(
        carla.Location(
            x=lead_spawn.location.x - 12,  # 12 meters behind the lead vehicle
            y=lead_spawn.location.y,
            z=lead_spawn.location.z
        ),
        lead_spawn.rotation  # Same orientation
    )

    # Spawn vehicles
    lead_vehicle = world.spawn_actor(lead_vehicle_bp, lead_spawn)
    ego_vehicle = world.spawn_actor(ego_vehicle_bp, ego_spawn)

    #LiDAR sensor attached to ego vehicle
    lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("range", "50.0")
    lidar_bp.set_attribute("channels", "32")
    lidar_bp.set_attribute("points_per_second", "100000")
    
    lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))  # Mount on top
    lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)

    
    tm = client.get_trafficmanager()
    tm_port = tm.get_port()
    
    
    lead_vehicle.set_autopilot(True, tm_port)
    ego_vehicle.set_autopilot(True, tm_port)
    
    
    tm.auto_lane_change(lead_vehicle, False)
    tm.auto_lane_change(ego_vehicle, False)


    # Start LiDAR sensor
    lidar_sensor.listen(lambda data: lidar_callback(data, ego_vehicle, lead_vehicle))

    try:
        while True:  
            time.sleep(1)  
    except KeyboardInterrupt:
        print("Simulation stopped manually.")

    finally:
        print("Destroying actors")
        lidar_sensor.destroy()
        ego_vehicle.destroy()
        lead_vehicle.destroy()

if __name__ == "__main__":
    main()
