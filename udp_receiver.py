import socket
import json

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on UDP port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)
    try:
        message = json.loads(data.decode())
        print(f"From {addr}: {message}")
    except json.JSONDecodeError:
        print(f"Received malformed data from {addr}")
