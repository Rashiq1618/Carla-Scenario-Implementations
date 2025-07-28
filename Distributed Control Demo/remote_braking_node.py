import socket
import json
import time

UDP_IP = "0.0.0.0"
UDP_PORT_RECEIVE = 9000
UDP_PORT_SEND = 9001
SENDER_IP = ""  # Computer 1 IP

recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((UDP_IP, UDP_PORT_RECEIVE))

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

braking_active = False

while True:
    try:
        msg, addr = recv_sock.recvfrom(1024)
        data = json.loads(msg.decode())

        speed = data.get("speed")
        state = data.get("traffic_light")

        print(f"Speed: {speed} km/h | Light: {state}")

        #brake logic for red light
        if not braking_active and state == "Red":
            send_sock.sendto(b"brake", (SENDER_IP, UDP_PORT_SEND))
            print("Brake message sent (light is RED).")
            braking_active = True

        #resume driving logic for green light
        if braking_active and state == "Green":
            send_sock.sendto(b"resume", (SENDER_IP, UDP_PORT_SEND))
            print("Resume message sent (light turned GREEN).")
            braking_active = False

    except Exception as e:
        print(f"Error: {e}")
        continue
