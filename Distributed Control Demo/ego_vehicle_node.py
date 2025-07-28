import carla
import socket
import json
import cv2
import numpy as np
import time

#UDP configurations
UDP_IP = ""  # Replace with Computer 2 IP
UDP_PORT_SEND = 9000
UDP_PORT_RECEIVE = 9001

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("", UDP_PORT_RECEIVE)) 
recv_sock.setblocking(False)

#connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()

#spawning the TESLA model 3
blueprint_lib = world.get_blueprint_library()
vehicle_bp = blueprint_lib.find('vehicle.tesla.model3')
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True)

#traffic manager
tm = client.get_trafficmanager()
tm.ignore_lights_percentage(vehicle, 100.0)  #ignores traffic lights

#Camera for live display
camera_bp = blueprint_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640')
camera_bp.set_attribute('image_size_y', '480')
camera_bp.set_attribute('fov', '90')


camera_transform = carla.Transform(
    carla.Location(x=-6, z=3),        # 6m behind, 3m above vehicle
    carla.Rotation(pitch=-10, yaw=0)  # slightly tilted down
)
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

#opencv
frame = None
def camera_callback(image):
    global frame
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (image.height, image.width, 4))
    frame = array[:, :, :3]  

camera.listen(lambda image: camera_callback(image))

#LOOOP
try:
    while True:
        if frame is not None:
            cv2.imshow("Third-Person RGB Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        #calculation of speed
        vel = vehicle.get_velocity()
        speed = 3.6 * (vel.x**2 + vel.y**2 + vel.z**2)**0.5  #m/s to  km/h

        #check for traffic light state
        tl = vehicle.get_traffic_light()
        state = "Unknown"
        if tl:
            state = tl.get_state().name

        #sending udp data
        data = {
            "speed": round(speed, 2),
            "traffic_light": state
        }
        send_sock.sendto(json.dumps(data).encode(), (UDP_IP, UDP_PORT_SEND))

        #receiving commands
        try:
            msg, _ = recv_sock.recvfrom(1024)
            cmd = msg.decode()
            if cmd == "brake":
                print("Brake message received.")
                vehicle.set_autopilot(False)
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            elif cmd == "resume":
                print("Resume message received.")
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))
                time.sleep(0.1)
                vehicle.set_autopilot(True)
        except BlockingIOError:
            pass

finally:
    camera.stop()
    vehicle.destroy()
    cv2.destroyAllWindows()
