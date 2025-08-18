import carla
import socket
import json
import cv2
import numpy as np
import time

#udp configs
UDP_IP = ""  # IP of Computer 2
UDP_PORT_SEND = 9000
UDP_PORT_RECEIVE = 9001

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("", UDP_PORT_RECEIVE))
recv_sock.setblocking(False)

#carla client 
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
blueprint_lib = world.get_blueprint_library()

# Traffic manager setup
tm = client.get_trafficmanager()
tm.set_synchronous_mode(False)
tm.set_global_distance_to_leading_vehicle(2.5)

#spawning ego
vehicle_bp = blueprint_lib.find('vehicle.tesla.model3')
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True, tm.get_port())

# Ignore lights and walkers in TM
tm.ignore_lights_percentage(vehicle, 100.0)
tm.ignore_walkers_percentage(vehicle, 100.0)

#cap velocity at 26km/h 
MAX_SPEED_KMH = 26.0
def cap_autopilot_speed(vehicle, tm, max_speed_kmh=MAX_SPEED_KMH):
    road_speed_limit = vehicle.get_speed_limit()  # km/h
    if road_speed_limit > 0:
        reduction = max(0.0, min(100.0, 100.0 * (1 - max_speed_kmh / road_speed_limit)))
        tm.vehicle_percentage_speed_difference(vehicle, reduction)
cap_autopilot_speed(vehicle, tm, MAX_SPEED_KMH)

# camera setup
camera_bp = blueprint_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640')
camera_bp.set_attribute('image_size_y', '480')
camera_bp.set_attribute('fov', '90')
camera_transform = carla.Transform(carla.Location(x=-6, z=3), carla.Rotation(pitch=-10))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

frame = None
def camera_callback(image):
    global frame
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (image.height, image.width, 4))
    frame = array[:, :, :3]

camera.listen(camera_callback)

#Pedestrian dectection using dot product
MAX_DETECTION_RANGE = 15.0  #values that are tested to work best 
BRAKE_RANGE = 6.0 #values that are tested to work best 
def detect_pedestrian(ego_transform, walkers):
    closest_distance = None
    for walker in walkers:
        loc = walker.get_location()
        direction = loc - ego_transform.location
        distance = direction.length()
        if distance > MAX_DETECTION_RANGE:
            continue
        forward_vector = ego_transform.get_forward_vector()
        direction = direction.make_unit_vector()
        dot = forward_vector.x * direction.x + forward_vector.y * direction.y + forward_vector.z * direction.z
        if dot > 0.7:
            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
    return (closest_distance is not None), closest_distance

#Loop
try:
    while True:
        if frame is not None:
            cv2.imshow("Third-Person Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        vel = vehicle.get_velocity()
        speed = 3.6 * (vel.x**2 + vel.y**2 + vel.z**2)**0.5

        ego_transform = vehicle.get_transform()
        walkers = world.get_actors().filter('walker.pedestrian.*')
        detected, distance = detect_pedestrian(ego_transform, walkers)

        #Sending detection data
        data = {
            "speed": round(speed, 2),
            "pedestrian_detected": detected,
            "distance": round(distance, 2) if distance else None
        }
        send_sock.sendto(json.dumps(data).encode(), (UDP_IP, UDP_PORT_SEND))

        try:
            msg, _ = recv_sock.recvfrom(1024)
            cmd_data = json.loads(msg.decode())

            cmd = cmd_data.get("cmd")
            dist = cmd_data.get("distance", None)

            if cmd == "brake":
                print("Brake message received.")
                vehicle.set_autopilot(False)
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

            elif cmd == "slowdown" and dist is not None:
                # Linear scaling of throttle
                throttle = 0.05 + (dist - BRAKE_RANGE) * (0.35 - 0.05) / (MAX_DETECTION_RANGE - BRAKE_RANGE)
                throttle = max(0.05, min(0.35, throttle))
                print(f"Slowdown to throttle={throttle:.2f} at distance={dist:.1f}m")
                vehicle.set_autopilot(True)
                cap_autopilot_speed(vehicle, tm, MAX_SPEED_KMH)  # Re-apply speed cap
                vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=0.0))

            elif cmd == "resume":
                print("Resume message received.")
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
                time.sleep(0.1)
                vehicle.set_autopilot(True, tm.get_port())
                cap_autopilot_speed(vehicle, tm, MAX_SPEED_KMH)  # Re-apply speed cap

        except BlockingIOError:
            pass

finally:
    camera.stop()
    vehicle.destroy()
    cv2.destroyAllWindows()
