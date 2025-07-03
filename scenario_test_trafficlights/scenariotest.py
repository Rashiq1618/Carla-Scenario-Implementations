import carla
import math
import time
import random
import socket
import json
import threading
import pygame
import numpy as np

def get_speed(vehicle):
    vel = vehicle.get_velocity()
    speed_mps = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
    return speed_mps * 3.6

def get_traffic_light_ahead(vehicle, distance_threshold=100.0):
    traffic_light = vehicle.get_traffic_light()
    if traffic_light and vehicle.is_at_traffic_light():
        tl_location = traffic_light.get_transform().location
        vehicle_location = vehicle.get_location()
        distance = vehicle_location.distance(tl_location)
        if 10.0 <= distance <= distance_threshold:
            return traffic_light, distance
    return None, None

def radar_callback(data, radar_data_list):
    radar_data_list.clear()
    for detection in data:
        radar_data_list.append(detection)

SEND_IP = "127.0.0.1"
SEND_PORT = 8000
RECV_PORT = 8001

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(('', RECV_PORT))
recv_sock.settimeout(0.01)

brake_flag = {"brake": False, "received": False}

def udp_receiver():
    while True:
        try:
            data, _ = recv_sock.recvfrom(1024)
            msg = json.loads(data.decode('utf-8'))
            if "brake" in msg:
                brake_flag["brake"] = msg["brake"]
                brake_flag["received"] = True
        except socket.timeout:
            continue
        except Exception:
            continue

threading.Thread(target=udp_receiver, daemon=True).start()

def process_camera_image(image, display_surface):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    display_surface.blit(surface, (0, 0))
    pygame.display.flip()

def main():
    pygame.init()
    display = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Drone Camera View")

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    spawn_points = world.get_map().get_spawn_points()
    ego_vehicle = world.spawn_actor(vehicle_bp, random.choice(spawn_points))
    ego_vehicle.set_autopilot(True)

    radar_bp = blueprint_library.find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', '30')
    radar_bp.set_attribute('vertical_fov', '5')
    radar_bp.set_attribute('range', '100')
    radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
    radar = world.spawn_actor(radar_bp, radar_transform, attach_to=ego_vehicle)
    radar_data_list = []
    radar.listen(lambda data: radar_callback(data, radar_data_list))

    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_transform = carla.Transform(carla.Location(x=-8.0, z=10.0), carla.Rotation(pitch=-25))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
    camera.listen(lambda image: process_camera_image(image, display))

    print("Simulation started")

    try:
        while True:
            world.tick()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            speed = get_speed(ego_vehicle)
            traffic_light, distance = get_traffic_light_ahead(ego_vehicle)
            tl_state_str = "None"

            if traffic_light and distance:
                state = traffic_light.state
                if state == carla.TrafficLightState.Red:
                    tl_state_str = "Red"
                elif state == carla.TrafficLightState.Yellow:
                    tl_state_str = "Yellow"
                elif state == carla.TrafficLightState.Green:
                    tl_state_str = "Green"

            msg = {
                "speed": speed,
                "traffic_light_state": tl_state_str,
                "distance_to_traffic_light": distance if distance else 0.0
            }
            send_sock.sendto(json.dumps(msg).encode('utf-8'), (SEND_IP, SEND_PORT))

            if brake_flag["received"]:
                if brake_flag["brake"]:
                    ego_vehicle.apply_control(carla.VehicleControl(brake=1.0))
                    print("Brake: True")
                else:
                    ego_vehicle.apply_control(carla.VehicleControl(brake=0.0))
                    print("Brake: False")

            time.sleep(0.1)

    finally:
        radar.stop()
        radar.destroy()
        camera.stop()
        camera.destroy()
        ego_vehicle.destroy()
        pygame.quit()
        print("Simulation ended")

if __name__ == '__main__':
    main()
