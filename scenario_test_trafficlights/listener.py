import socket
import json

RECEIVE_IP = "0.0.0.0"
RECEIVE_PORT = 8000  

SEND_IP = "127.0.0.1" 
SEND_PORT = 8001       

recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((RECEIVE_IP, RECEIVE_PORT))
recv_sock.settimeout(0.1)

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def decide_brake(traffic_light):
    if traffic_light == "Red":
        return True
    elif traffic_light in ["Green", "Yellow"]:
        return False
    return False

while True:
    try:
        data, _ = recv_sock.recvfrom(4096)
        msg = json.loads(data.decode('utf-8'))

        speed = msg.get("speed", 0.0)
        traffic_light = msg.get("traffic_light_state", "None")
        distance = msg.get("distance_to_traffic_light", 0.0)

        print(f"Speed: {speed:.2f} km/h | Traffic Light: {traffic_light} | Distance: {distance:.2f} m")

        brake = decide_brake(traffic_light)
        control_msg = {"brake": brake}
        send_sock.sendto(json.dumps(control_msg).encode('utf-8'), (SEND_IP, SEND_PORT))

    except socket.timeout:
        continue
    except Exception:
        continue
