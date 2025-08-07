import socket
import json
import time

#udp configs
UDP_IP = "0.0.0.0"
UDP_PORT_RECEIVE = 9000
UDP_PORT_SEND = 9001
CARLA_IP = ""  #IP computer 1
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((UDP_IP, UDP_PORT_RECEIVE))

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

braking_active = False
brake_sent_time = None  #tracks when brake msg was sent

#Main LOOP
while True:
    try:
        msg, addr = recv_sock.recvfrom(1024)
        data = json.loads(msg.decode())

        speed = data.get("speed")
        detected = data.get("pedestrian_detected")
        distance = data.get("distance")

        print(f"Speed: {speed} km/h | Pedestrian: {detected} | Distance: {distance} m")

        #Brake logic
        if detected and not braking_active:
            send_sock.sendto(b"brake", (CARLA_IP, UDP_PORT_SEND))
            print("Brake message sent (pedestrian detected).")
            braking_active = True
            brake_sent_time = time.time()

        #Resume logic with 10s delay 
        if braking_active and not detected and brake_sent_time:
            current_time = time.time()
            if current_time - brake_sent_time >= 10:
                send_sock.sendto(b"resume", (CARLA_IP, UDP_PORT_SEND))
                print("Resume message sent (10s after pedestrian cleared).")
                braking_active = False
                brake_sent_time = None

    except Exception as e:
        print(f"Error: {e}")
        continue
