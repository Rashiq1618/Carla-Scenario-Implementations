import carla
import random
import pygame
import numpy as np
import math
import time

#vars
TARGET_SPEED = 25.0  
WAYPOINT_DISTANCE = 3.0  
LOOKAHEAD_DIST = 5.0  
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 600

def get_speed(vehicle):
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

def draw_waypoints_debug(world, waypoints):
    for wp in waypoints:
        world.debug.draw_point(wp.transform.location + carla.Location(z=0.5),
                               size=0.2,
                               color=carla.Color(0, 255, 0),
                               life_time=0.0,
                               persistent_lines=True)

def get_steering_angle(vehicle, target_location):
    transform = vehicle.get_transform()
    location = transform.location
    yaw = math.radians(transform.rotation.yaw)
    
    v_vec = np.array([math.cos(yaw), math.sin(yaw)])
    t_vec = np.array([target_location.x - location.x,
                      target_location.y - location.y])
    t_vec_norm = np.linalg.norm(t_vec)
    if t_vec_norm == 0:
        return 0.0
    t_vec = t_vec / t_vec_norm
    dot = np.clip(np.dot(v_vec, t_vec), -1.0, 1.0)
    angle = math.acos(dot)
    cross = np.cross(v_vec, t_vec)
    if cross < 0:
        angle *= -1
    return angle / (math.pi / 2)  

def main():
    pygame.init()
    display = pygame.display.set_mode((DISPLAY_WIDTH, DISPLAY_HEIGHT))
    pygame.display.set_caption("Waypoint Follow")
    font = pygame.font.SysFont(None, 36)  

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    map = world.get_map()
    blueprint_library = world.get_blueprint_library()

    
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()

    spawn_point = random.choice(map.get_spawn_points())
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    #Creatiing and attaching a camera to the vehicle
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(DISPLAY_WIDTH))
    camera_bp.set_attribute('image_size_y', str(DISPLAY_HEIGHT))
    camera_bp.set_attribute('fov', '100')
    cam_transform = carla.Transform(carla.Location(x=-5, z=2.5), carla.Rotation(pitch=0, yaw=0))
    camera = world.spawn_actor(camera_bp, cam_transform, attach_to=vehicle)

    camera_image = [None]

    def process_img(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3]
        array = array[:, :, ::-1]  
        array = np.fliplr(array)  
        camera_image[0] = array

    camera.listen(process_img)

    
    waypoints = []
    wp = map.get_waypoint(spawn_point.location)
    for _ in range(120):  
        next_wps = wp.next(WAYPOINT_DISTANCE)
        if next_wps:
            wp = next_wps[0]
            waypoints.append(wp)
        else:
            break

    draw_waypoints_debug(world, waypoints)

    clock = pygame.time.Clock()
    waypoint_index = 0
    arrived = False 

    try:
        while True:
            clock.tick(30)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            if len(waypoints) == 0:
                continue

            vehicle_loc = vehicle.get_location()

            # moving to next waypoint if close
            while waypoint_index < len(waypoints) - 1 and \
                  vehicle_loc.distance(waypoints[waypoint_index].transform.location) < LOOKAHEAD_DIST:
                waypoint_index += 1

            target_wp = waypoints[min(waypoint_index, len(waypoints) - 1)]
            target_loc = target_wp.transform.location
            speed = get_speed(vehicle)

            if not arrived and waypoint_index >= len(waypoints) - 1 and \
               vehicle_loc.distance(target_loc) < 2.0:
                # Final waypoint reached
                control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
                arrived = True
            elif not arrived:
                steer = get_steering_angle(vehicle, target_loc)
                throttle = 0.5 if speed < TARGET_SPEED else 0.0
                brake = 0.3 if speed > TARGET_SPEED + 5 else 0.0
                control = carla.VehicleControl(
                    throttle=throttle,
                    steer=np.clip(steer, -1.0, 1.0),
                    brake=brake
                )
            else:
                control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)

            vehicle.apply_control(control)

            
            if camera_image[0] is not None:
                surface = pygame.surfarray.make_surface(np.rot90(camera_image[0]))
                display.blit(surface, (0, 0))

                speed_text = f"Speed: {speed:.1f} km/h"
                text_surface = font.render(speed_text, True, (255, 255, 255))
                display.blit(text_surface, (10, 10))

                pygame.display.flip()

    finally:
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        waypoints.clear()
        pygame.quit()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Simulation Stopped.")
