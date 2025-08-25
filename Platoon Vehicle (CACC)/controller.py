import socket
import json
import time
from collections import deque

#Configs
CARLA_CLIENT_IP = "" #Carla client IP
LISTEN_PORT = 9001
SEND_PORT = 9000

#Controller gains
Kp = 1.2  # Proportional gain for distance error
Kd = 1.5  # Derivative gain for relative speed
Kff = 1.0 # Feed-forward gain for front car's acceleration
DISTANCE_BETWEEN = 10.0 # Target distance in meters
dt = 0.05


SPEED_HISTORY_SIZE = 5
speed_histories = {}

#Stopping Behavior Thresholds
STOPPED_SPEED_THRESHOLD = 0.5  # Speeds below this (in km/h) are considered "stopped"
DISTANCE_TOLERANCE = 0.25      # Allowed distance error (in meters) when stopped

def kmh_to_ms(v_kmh):
    """Converts speed from km/h to m/s."""
    return v_kmh / 3.6

def ms_to_kmh(v_ms):
    """Converts speed from m/s to km/h."""
    return v_ms * 3.6

def main():
    print("Starting External CACC Controller")
    recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_socket.bind(('', LISTEN_PORT))
    recv_socket.setblocking(0)
    print(f"Listening for vehicle state on UDP port {LISTEN_PORT}")

    send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Sending control commands to {CARLA_CLIENT_IP}:{SEND_PORT}")

    try:
        while True:
            try:
                data, addr = recv_socket.recvfrom(4096)
                vehicle_states = json.loads(data.decode('utf-8'))

                print("\nController: Received Vehicle States ---")
                for state in vehicle_states:
                    print(f"  ID: {state['id']}, Speed: {state['speed']:.2f} km/h, DistToFront: {state['dist_to_front']:.2f} m")

                #Updating speed histories for acceleration calculation
                for state in vehicle_states:
                    vehicle_id = state['id']
                    if vehicle_id not in speed_histories:
                        speed_histories[vehicle_id] = deque([state['speed']] * SPEED_HISTORY_SIZE, maxlen=SPEED_HISTORY_SIZE)
                    else:
                        speed_histories[vehicle_id].append(state['speed'])

                control_commands = {}
                for i in range(1, len(vehicle_states)):
                    follower_state = vehicle_states[i]
                    front_state = vehicle_states[i-1]

                    foll_id = follower_state['id']
                    v_foll_ms = kmh_to_ms(follower_state['speed'])
                    front_id = front_state['id']
                    v_front_kmh = front_state['speed']
                    
                    dist = follower_state['dist_to_front']
                    error = dist - DISTANCE_BETWEEN
                    
                    
                    # If the front car is effectively stopped, switch to a position controller
                    # to finely adjust distance
                    if v_front_kmh < STOPPED_SPEED_THRESHOLD:
                        # If we are within the desired distance tolerance, command a full stop.
                        if abs(error) < DISTANCE_TOLERANCE:
                            target_speed_kmh = 0.0
                        else:
                            # Otherwise, use a simple proportional controller on the distance error
                            # to generate a slow "creep" speed to fix the gap
                            position_kp = 0.8  # Using a gentle gain for creeping
                            creep_speed_ms = position_kp * error
                            # Cap the creep speed to prevent lurching
                            max_creep_speed_ms = kmh_to_ms(3.0) # Max 3 km/h creep
                            creep_speed_ms = max(-max_creep_speed_ms, min(creep_speed_ms, max_creep_speed_ms))
                            
                            # We don't allow reversing, so speed is at least 0
                            target_speed_kmh = ms_to_kmh(max(0.0, creep_speed_ms))
                    else:
                        # CACC LOGIC
                        rel_speed_ms = kmh_to_ms(v_front_kmh) - v_foll_ms

                        # Calculate front car's acceleration
                        a_front_ms2 = 0.0
                        if front_id in speed_histories and len(speed_histories[front_id]) == SPEED_HISTORY_SIZE:
                            history = speed_histories[front_id]
                            # Average of first 2 vs last 2 points for a smoother derivative
                            v_front_avg_old = sum(list(history)[0:2]) / 2.0
                            v_front_avg_new = sum(list(history)[-2:]) / 2.0
                            a_front_ms2 = (kmh_to_ms(v_front_avg_new) - kmh_to_ms(v_front_avg_old)) / (dt * (SPEED_HISTORY_SIZE / 2))

                        # Full CACC control law
                        v_des_ms = kmh_to_ms(v_front_kmh) + (Kp * error) + (Kd * rel_speed_ms) + (Kff * a_front_ms2 * dt)
                        target_speed_kmh = ms_to_kmh(max(0.0, v_des_ms))
                    
                    control_commands[foll_id] = target_speed_kmh

                if control_commands:
                    print("Controller: Sending Control Commands")
                    print(f"  {control_commands}")
                    message = json.dumps(control_commands).encode('utf-8')
                    send_socket.sendto(message, (CARLA_CLIENT_IP, SEND_PORT))

            except BlockingIOError:
                pass
            except Exception as e:
                print(f"An error occurred: {e}")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nController stopped by user.")
    finally:
        recv_socket.close()
        send_socket.close()

if __name__ == "__main__":
    main()
