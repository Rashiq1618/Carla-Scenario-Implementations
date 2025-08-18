import socket
import json
import time

#udp configs
UDP_IP = "0.0.0.0"
UDP_PORT_RECEIVE = 9000
UDP_PORT_SEND = 9001
CARLA_IP = ""

recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((UDP_IP, UDP_PORT_RECEIVE))
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

braking_active = False
slowdown_active = False
brake_sent_time = None

BRAKE_RANGE = 6.0
SLOWDOWN_RANGE = 15.0
RESUME_DELAY = 10  

while True:
    try:
        msg, addr = recv_sock.recvfrom(1024)
        data = json.loads(msg.decode())

        speed = data.get("speed")
        detected = data.get("pedestrian_detected")
        distance = data.get("distance")

        print(f"Speed: {speed} km/h | Pedestrian: {detected} | Distance: {distance} m")

        if detected and distance is not None:
            #Brake
            if distance <= BRAKE_RANGE and not braking_active:
                payload = json.dumps({"cmd": "brake"}).encode()
                send_sock.sendto(payload, (CARLA_IP, UDP_PORT_SEND))
                print("Brake message sent.")
                braking_active = True
                slowdown_active = False
                brake_sent_time = time.time()

            #slowdown
            elif BRAKE_RANGE < distance <= SLOWDOWN_RANGE and not braking_active:
                payload = json.dumps({"cmd": "slowdown", "distance": distance}).encode()
                send_sock.sendto(payload, (CARLA_IP, UDP_PORT_SEND))
                print("Slowdown message sent.")


        #Resume
        if not detected and (braking_active):
            if braking_active:
                if brake_sent_time and time.time() - brake_sent_time >= RESUME_DELAY:
                    payload = json.dumps({"cmd": "resume"}).encode()
                    send_sock.sendto(payload, (CARLA_IP, UDP_PORT_SEND))
                    print("Resume message sent (after brake).")
                    braking_active = False
                    brake_sent_time = None
            else:
                payload = json.dumps({"cmd": "resume"}).encode()
                send_sock.sendto(payload, (CARLA_IP, UDP_PORT_SEND))
                print("Resume message sent (after slowdown).")
                slowdown_active = False

    except Exception as e:
        print(f"Error: {e}")
        continue
