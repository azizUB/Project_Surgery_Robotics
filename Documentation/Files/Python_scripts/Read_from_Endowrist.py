import socket
import json
import time

UDP_IP = "0.0.0.0"  # Listen on your computer's IP
UDP_PORT = 12345
BUFFER_SIZE = 1024

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
#print(f"Listening on {UDP_IP}:{UDP_PORT}")

message_counter = 0

try:
    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        message = data.decode()  # Decode the received message
        #print(f"Received message from {addr}: {message}")
        try:
            orientation = json.loads(data.decode())
            device_id = orientation.get("device")
            roll = orientation.get("roll")
            pitch = orientation.get("pitch")
            yaw = orientation.get("yaw")

            if device_id=="G1_Endo": 
                message_counter += 1
                if message_counter % 10 == 0:
                    roll1 = roll
                    pitch1 = pitch
                    yaw1 = yaw
                    print(f"Data from {device_id} (every 10th message): Roll_1: {roll1:.0f}, Pitch_1: {pitch1:.0f}, Yaw_1: {yaw1:.0f}")
            #time.sleep(0.001)
        except json.JSONDecodeError:
                print("Invalid JSON data received")
except KeyboardInterrupt:
    print('You pressed Ctrl+C!')
    sock.close()
    print("Socket closed.")

print("Program terminated.")