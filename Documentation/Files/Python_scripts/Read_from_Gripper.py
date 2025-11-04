import socket
import json
import time

UDP_IP = "0.0.0.0"  # Escoltar a totes les interfícies
UDP_PORT = 12345
BUFFER_SIZE = 1024

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Listening for gripper data on {UDP_IP}:{UDP_PORT}")

message_counter = 0
yaw_ref = None

try:
    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        try:
            message = data.decode()
            gripper_data = json.loads(message)
            device_id = gripper_data.get("device")

            if device_id == "G1_Gri":
                roll = gripper_data.get("roll")
                pitch = gripper_data.get("pitch")
                yaw = gripper_data.get("yaw")
                s1 = gripper_data.get("s1")
                s2 = gripper_data.get("s2")
                
                if yaw_ref is None:
                    yaw_ref = yaw
                
                yaw_rel = yaw - yaw_ref
                
                message_counter += 1
                if message_counter % 10 == 0:
                    print(f"[{device_id}] Roll: {roll:.0f}, Pitch: {pitch:.0f}, Yaw: {yaw:.0f}, Yaw_rel : {yaw_rel:.0f}, S1: {s1}, S2: {s2}")
            # else:
            #     print("Message from another device or missing device ID.")

        except json.JSONDecodeError:
            print("Invalid JSON data received")

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    sock.close()
    print("Socket closed.")
