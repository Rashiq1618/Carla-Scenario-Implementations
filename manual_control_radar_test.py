import carla
import numpy as np
import pygame
import time
import random
import math
import os
import csv
from datetime import datetime

# Emergency braking thresholds
BRAKE_DISTANCE = 10.0
FOLLOW_DISTANCE = 15.0
BRAKE_FORCE = 1.0
SPEED_LIMIT = 45.0  # km/h

# Globals
obstacle_distance = None
camera_surface = None
show_radar = False
collision_log = []
data_logging = True

#unique CSV file to log data
save_dir = "ego_vehicle_data"
os.makedirs(save_dir, exist_ok=True)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = os.path.join(save_dir, f"ego_data_{timestamp}.csv")

with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Speed (km/h)", "", "Brake", "", "Emergency Brake Applied", "", "Obstacle Distance (m)"])

def log_ego_data(speed, brake, emergency_brake, obstacle):
    with open(csv_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            f"{speed:.1f}", "", 
            f"{brake:.2f}", "", 
            "Yes" if emergency_brake else "No", "", 
            f"{obstacle:.1f}" if obstacle is not None else "N/A"
        ])

def radar_callback(radar_data):
    global obstacle_distance
    obstacle_distance = None

    ego_transform = radar_data.transform
    min_front_distance = float("inf")

    for detection in radar_data:
        relative_pos = carla.Location(x=detection.depth)
        world_pos = ego_transform.transform(relative_pos)
        ego_location = ego_transform.location
        ego_rotation = ego_transform.rotation

        yaw = math.radians(ego_rotation.yaw)
        forward_vector = np.array([math.cos(yaw), math.sin(yaw)])
        right_vector = np.array([-math.sin(yaw), math.cos(yaw)])

        rel_vector = np.array([
            world_pos.x - ego_location.x,
            world_pos.y - ego_location.y
        ])

        x_local = np.dot(rel_vector, forward_vector)
        y_local = np.dot(rel_vector, right_vector)

        if x_local > 0 and detection.depth < min_front_distance:
            min_front_distance = detection.depth

    if min_front_distance < BRAKE_DISTANCE:
        obstacle_distance = min_front_distance

def camera_callback(image):
    global camera_surface
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))[:, :, :3]
    array = array[:, :, ::-1]
    array = np.rot90(array)
    camera_surface = pygame.surfarray.make_surface(array)

def spawn_autopilot_vehicles(world, blueprint_library, ego_spawn, count=5, tm_port=8000):
    vehicle_bps = blueprint_library.filter('vehicle.*')
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    spawned = []

    for spawn in spawn_points:
        if len(spawned) >= count:
            break
        if spawn.location.distance(ego_spawn.location) > 10:
            bp = random.choice(vehicle_bps)
            vehicle = world.try_spawn_actor(bp, spawn)
            if vehicle:
                vehicle.set_autopilot(True, tm_port)
                spawned.append(vehicle)
    return spawned

def get_speed(vehicle):
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

def draw_info_panel(surface, ego_vehicle, control, reverse_mode, recording):
    panel_width = 200
    panel_rect = pygame.Rect(800, 0, panel_width, 600)
    pygame.draw.rect(surface, (30, 30, 30), panel_rect)

    font_title = pygame.font.SysFont("Arial", 24, bold=True)
    font_data = pygame.font.SysFont("Arial", 20)

    y = 20

    def draw_line(label, value):
        nonlocal y
        label_surface = font_title.render(label, True, (255, 255, 255))
        value_surface = font_data.render(value, True, (180, 180, 180))
        surface.blit(label_surface, (810, y))
        surface.blit(value_surface, (810, y + 25))
        y += 60

    speed = get_speed(ego_vehicle)
    draw_line("Speed", f"{speed:.1f} km/h")
    draw_line("Throttle", f"{control.throttle:.2f}")
    draw_line("Brake", f"{control.brake:.2f}")
    draw_line("Steer", f"{control.steer:.2f}")
    draw_line("Reverse", "Yes" if reverse_mode else "No")
    draw_line("Gear", str(ego_vehicle.get_control().gear))
    if obstacle_distance is not None:
        draw_line("Obstacle", f"{obstacle_distance:.1f} m")

    if recording:
        rec_font = pygame.font.SysFont("Arial", 22, bold=True)
        rec_text = rec_font.render("● Recording", True, (255, 0, 0))
        surface.blit(rec_text, (810, 570))

def collision_callback(event):
    actor_type = event.other_actor.type_id
    collision_log.append(f"Collision with {actor_type} at frame {event.frame}")
    print(f"Collision with {actor_type} at frame {event.frame}")

def main():
    global show_radar
    pygame.init()
    display = pygame.display.set_mode((1000, 600))
    pygame.display.set_caption("Ego Vehicle Dashboard")
    clock = pygame.time.Clock()

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    ego_bp = blueprint_library.find("vehicle.audi.tt")
    lead_bp = blueprint_library.find("vehicle.tesla.model3")

    spawn_points = world.get_map().get_spawn_points()
    if len(spawn_points) < 2:
        print("Not enough spawn points")
        return

    lead_spawn = spawn_points[0]
    ego_spawn = carla.Transform(
        carla.Location(
            x=lead_spawn.location.x - 12,
            y=lead_spawn.location.y,
            z=lead_spawn.location.z
        ),
        lead_spawn.rotation
    )

    lead_vehicle = world.try_spawn_actor(lead_bp, lead_spawn)
    ego_vehicle = world.try_spawn_actor(ego_bp, ego_spawn)

    tm = client.get_trafficmanager()
    tm_port = tm.get_port()
    lead_vehicle.set_autopilot(True, tm_port)

    radar_bp = blueprint_library.find("sensor.other.radar")
    radar_bp.set_attribute("horizontal_fov", "30.0")
    radar_bp.set_attribute("vertical_fov", "5.0")
    radar_bp.set_attribute("range", "50.0")
    radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
    radar_sensor = world.spawn_actor(radar_bp, radar_transform, attach_to=ego_vehicle)
    radar_sensor.listen(lambda data: radar_callback(data))

    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", "800")
    camera_bp.set_attribute("image_size_y", "600")
    camera_bp.set_attribute("fov", "90")
    camera_transform = carla.Transform(
        carla.Location(x=-6.0, z=3.0),
        carla.Rotation(pitch=-15)
    )
    camera_sensor = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
    camera_sensor.listen(lambda image: camera_callback(image))

    collision_bp = blueprint_library.find("sensor.other.collision")
    collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego_vehicle)
    collision_sensor.listen(lambda event: collision_callback(event))

    other_vehicles = spawn_autopilot_vehicles(world, blueprint_library, ego_spawn, count=5, tm_port=tm_port)

    try:
        reverse_mode = False
        while True:
            clock.tick(30)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        reverse_mode = not reverse_mode
                        print("Reverse mode:", reverse_mode)
                    elif event.key == pygame.K_r:
                        show_radar = not show_radar

            keys = pygame.key.get_pressed()
            control = carla.VehicleControl()
            control.reverse = reverse_mode

            emergency_brake = False
            speed = get_speed(ego_vehicle)

            if obstacle_distance is not None and obstacle_distance < BRAKE_DISTANCE and not reverse_mode:
                control.throttle = 0.0
                control.steer = 0.0
                speed_factor = speed / 52.0
                distance_factor = max(0.1, obstacle_distance / BRAKE_DISTANCE)
                scaled_brake = min(1.0, speed_factor / distance_factor)
                control.brake = scaled_brake
                emergency_brake = True
            else:
                if keys[pygame.K_w]:
                    control.throttle = 0.552
                if keys[pygame.K_s]:
                    control.brake = 0.5
                if keys[pygame.K_d]:
                    control.steer = -0.3
                if keys[pygame.K_a]:
                    control.steer = 0.3

                if obstacle_distance is not None and obstacle_distance < FOLLOW_DISTANCE and not reverse_mode:
                    control.throttle = min(control.throttle, 0.3)

            if speed > SPEED_LIMIT:
                control.throttle = 0.0

            ego_vehicle.apply_control(control)

            if data_logging:
                log_ego_data(speed, control.brake, emergency_brake, obstacle_distance)

            if camera_surface:
                display.blit(camera_surface, (0, 0))
                draw_info_panel(display, ego_vehicle, control, reverse_mode, data_logging)
                pygame.display.flip()

    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        for actor in [radar_sensor, camera_sensor, collision_sensor, ego_vehicle, lead_vehicle] + other_vehicles:
            if actor and actor.is_alive:
                actor.destroy()
        pygame.quit()

if __name__ == "__main__":
    main()
#perfecto