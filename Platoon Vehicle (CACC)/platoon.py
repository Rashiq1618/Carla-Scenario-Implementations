import carla
import random
import time
import sys
import socket
import json
import numpy as np
import cv2

sys.path.append(r"C:\Users\Carla\PythonAPI")

from agents.navigation.controller import VehiclePIDController
from agents.tools.misc import get_speed

#Configs
NUM_VEHICLES = 3
DISTANCE_BETWEEN = 10.0

#UDP configs
CONTROLLER_IP = "192.168.178.101"
LISTEN_PORT = 9000
SEND_PORT = 9001


SPEED_THRESHOLD_FOR_ZERO = 0.1 # km/h

class CameraManager:
    """ Manages the camera sensor and processes its image data. """
    def __init__(self):
        self.image = None

    def process_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        self.image = array[:, :, :3]  


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    vehicles = []
    camera = None
    
    #UDP setup
    recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_socket.bind(('', LISTEN_PORT))
    recv_socket.setblocking(0)
    send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    camera_manager = CameraManager()

    try:
        ws = world.get_settings()
        dt = ws.fixed_delta_seconds if ws.synchronous_mode and ws.fixed_delta_seconds else 0.05
        
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter("model3")[0]

        spawn_points = world.get_map().get_spawn_points()
        spawn_point = random.choice(spawn_points)

        # Spawn platoon vehicles
        for i in range(NUM_VEHICLES):
            if i == 0:
                vehicle = world.spawn_actor(vehicle_bp, spawn_point)
                vehicle.set_autopilot(True)
            else:
                forward_vector = spawn_point.get_forward_vector()
                spawn_loc = spawn_point.location - (forward_vector * DISTANCE_BETWEEN * i)
                spawn_transform = carla.Transform(spawn_loc, spawn_point.rotation)
                vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
            vehicles.append(vehicle)

        #Cam setup
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '110')

        middle_vehicle = vehicles[len(vehicles) // 2]
        initial_loc = middle_vehicle.get_location()
        camera_transform = carla.Transform(
            carla.Location(x=initial_loc.x, y=initial_loc.y, z=25),  
            carla.Rotation(pitch=-90)
        )
        camera = world.spawn_actor(camera_bp, camera_transform)  
        camera.listen(camera_manager.process_image)

        # PID controllers for followers
        controllers = []
        for i in range(1, NUM_VEHICLES):
            args_lateral = {"K_P": 1.0, "K_D": 0.02, "K_I": 0.0, "dt": dt}
            args_longitudinal = {"K_P": 1.0, "K_D": 0.05, "K_I": 0.0, "dt": dt}
            controller = VehiclePIDController(vehicles[i], args_lateral=args_lateral, args_longitudinal=args_longitudinal)
            controllers.append(controller)

        print(f"Spawned {len(vehicles)} vehicles. Waiting for external controller.")
        
        target_speeds_kmh = {vehicles[i].id: 0.0 for i in range(1, NUM_VEHICLES)}
        tick_count = 0

        
        prev_loc = camera.get_transform().location
        alpha = 0.1  

        while True:
            world.tick()

            #Camera adjustments
            mid_loc = middle_vehicle.get_location()
            target_loc = carla.Location(x=mid_loc.x, y=mid_loc.y, z=25) 
            smooth_loc = carla.Location(
                x = prev_loc.x + alpha * (target_loc.x - prev_loc.x),
                y = prev_loc.y + alpha * (target_loc.y - prev_loc.y),
                z = prev_loc.z + alpha * (target_loc.z - prev_loc.z)
            )
            camera.set_transform(carla.Transform(smooth_loc, carla.Rotation(pitch=-90)))
            prev_loc = smooth_loc

            # Display camera feed
            if camera_manager.image is not None:
                cv2.imshow('Platoon SwagCAM', camera_manager.image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # 1. GATHER AND SEND VEHICLE STATES
            vehicle_states = []
            for i, v in enumerate(vehicles):
                dist_to_front = v.get_location().distance(vehicles[i-1].get_location()) if i > 0 else 0.0
                
                current_speed = get_speed(v)
                if current_speed < SPEED_THRESHOLD_FOR_ZERO:
                    current_speed = 0.0

                state = {'id': v.id, 'speed': current_speed, 'dist_to_front': dist_to_front}
                vehicle_states.append(state)
            
            message = json.dumps(vehicle_states).encode('utf-8')
            send_socket.sendto(message, (CONTROLLER_IP, SEND_PORT))
            
            if tick_count % 40 == 0:
                 print(f"\n[CARLA_CLIENT] Sent state update for {len(vehicle_states)} vehicles.")

            # 2. RECEIVE AND PARSE CONTROL COMMANDS
            try:
                data, addr = recv_socket.recvfrom(1024)
                commands = json.loads(data.decode('utf-8'))
                if tick_count % 40 == 0:
                    print(f"[CARLA_CLIENT] Received control commands: {commands}")
                for foll_id_str, speed in commands.items():
                    target_speeds_kmh[int(foll_id_str)] = speed
            except BlockingIOError:
                pass
            except Exception as e:
                print(f"Error receiving/parsing commands: {e}")

            # 3. APPLY CONTROL TO FOLLOWERS
            for i in range(1, NUM_VEHICLES):
                follower = vehicles[i]
                controller = controllers[i-1]
                waypoint = world.get_map().get_waypoint(vehicles[i-1].get_location())
                target_speed = target_speeds_kmh.get(follower.id, 0.0)
                control = controller.run_step(target_speed, waypoint)
                follower.apply_control(control)

            tick_count += 1
            time.sleep(dt)

    finally:
        print("Destroying actors...")
        if camera and camera.is_alive:
            camera.destroy()
        for v in reversed(vehicles):
            if v and v.is_alive:
                v.destroy()
        
        recv_socket.close()
        send_socket.close()
        cv2.destroyAllWindows()
        print("Done.")

if __name__ == "__main__":
    main()
