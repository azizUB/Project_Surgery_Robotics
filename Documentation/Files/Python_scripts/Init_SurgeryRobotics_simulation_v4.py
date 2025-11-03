from robodk.robolink import *
from robodk.robomath import *
import time
import math
import tkinter as tk
import threading
import socket
import json

# Constants
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 1024
ROBOT_NAME = 'UR5e'
ZERO_YAW_TOOL = 0
ZERO_YAW_GRIPPER = 0
READ_INTERVAL_S = 0.01

Endowrist_rpy = None
Gripper_rpy = None
Servo_torques = None
data_lock = threading.Lock()# semaphor to manage data from 2 threads

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
#print(f"Listening on {UDP_IP}:{UDP_PORT}")

# Initialize RoboDK
def initialize_robodk():
    RDK = Robolink()
    robot = RDK.Item(ROBOT_NAME)
    base = RDK.Item(f'{ROBOT_NAME} Base')
    endowrist = RDK.Item('Endowrist')
    gripper = RDK.Item('Gripper')
    needle = RDK.Item('Needle')
    Init_target = RDK.Item('Init')
    robot.setPoseFrame(base)
    robot.setPoseTool(endowrist)
    gripper_init = TxyzRxyz_2_Pose([0, 5, -105, 0, 0, 0])
    gripper.setParent(endowrist)
    gripper.setPose(gripper_init)
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])
    needle.setParent(gripper)
    needle.setPose(needle_init)
    robot.MoveL(Init_target)
    robot.setSpeed(50)
    return robot, base, gripper, needle

# Transformation Endowrist to base
def endowrist2base_orientation(roll, pitch, yaw):
    roll2 = (roll + 90) % 360
    pitch2 = pitch % 360
    yaw2 = yaw % 360
    return roll2, pitch2, yaw2

# Function to update the label with text
def update_text_label(label, tool_orientation, gripper_orientation, status_message, torque_values):
    full_text = f"Tool orientation: {tool_orientation}\nGripper orientation: {gripper_orientation}\n{status_message}\nTorque Values: {torque_values}"
    label.after(0, lambda: label.config(text=full_text))

# Function to read UDP data and update the global variable
def read_data_UDP():
    global Endowrist_rpy, Gripper_rpy, Servo_torques, data_lock
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE) 
            try:
                received_data = json.loads(data.decode())
                device_id = received_data.get("device")
                if device_id == "G1_Endo":
                    with data_lock:
                        Endowrist_rpy = received_data
                elif device_id == "G1_Gri":
                    with data_lock:
                        Gripper_rpy = received_data
                elif device_id == "G1_Servos":
                    with data_lock:
                        Servo_torques = received_data
            except json.JSONDecodeError:
                print("Error decoding JSON data")
        except socket.error as e:
            #print(f"Socket error in UDP reader: {e}")
            sock.close()
            print("Socket closed.")
            break

# Function to process the latest UDP data and move the robot
def move_robot(robot, gripper, needle, text_label, torque_button): # Hemos añadido el argumento torque_button
    global ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, Endowrist_rpy, Gripper_rpy, data_lock
    global e_roll, e_pitch, e_yaw, g_roll, g_pitch, g_yaw, s1, s2, s3, s4
    
    endowrist_orientation_msg = ""
    gripper_orientation_msg = ""
    status_message = ""
    servo_torques_msg = ""
    
    while True:
        with data_lock:
            current_Endowrist_rpy = Endowrist_rpy
            current_Gripper_rpy = Gripper_rpy

        if current_Endowrist_rpy:
            e_roll = Endowrist_rpy.get("roll")
            e_pitch = Endowrist_rpy.get("pitch")
            e_yaw = Endowrist_rpy.get("yaw")
            s3 = Endowrist_rpy.get("s3")
            s4 = Endowrist_rpy.get("s4")
            endo_roll, endo_pitch, endo_yaw = endowrist2base_orientation(e_roll, e_pitch, e_yaw)
            #print(f"Endowrist: {endo_roll}, {endo_pitch}, {endo_yaw}")
            # Move Endowrist
            endowrist_pose = robot.Pose()
            Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
            endowrist_pose_new = transl(Xr, Yr, Zr) * rotz(math.radians(ZERO_YAW_TOOL)) * rotz(math.radians(endo_yaw)) * roty(math.radians(endo_pitch)) * rotx(math.radians(endo_roll))
            if robot.MoveL_Test(robot.Joints(), endowrist_pose_new) == 0:
                robot.MoveL(endowrist_pose_new, True)
                endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                status_message = ""
            else:
                endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                status_message = "Robot cannot reach the position"
                
            if s3 == 0 or s4 == 0:
                current_pose = robot.Pose()
                # Z movement based on S3 and S4 buttons
                Tz = transl(0, 0, 5) if s3 == 0 else transl(0, 0, -5)
                new_pose = current_pose * Tz  # translació relativa

                status_message = "⬆ Botó S3 premut: pujant" if s3 == 0 else "⬇ Botó S4 premut: baixant"

                if robot.MoveL_Test(robot.Joints(), new_pose) == 0:
                    robot.MoveL(new_pose, True)
                else:
                    status_message = "❌ No es pot moure més en Z (relatiu)"
                    
        if current_Gripper_rpy:
            g_roll = Gripper_rpy.get("roll") - Endowrist_rpy.get("roll")
            g_pitch = Gripper_rpy.get("pitch") - Endowrist_rpy.get("pitch")
            g_yaw = Gripper_rpy.get("yaw") - Endowrist_rpy.get("yaw")
            s1 = Gripper_rpy.get("s1")
            s2 = Gripper_rpy.get("s2")
            #print(f"Gripper: {g_roll}, {g_pitch}, {g_yaw}")
            # Move Gripper
            gripper_pose = gripper.Pose()
            Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)
            gripper_pose_new = transl(Xg, Yg, Zg) * rotz(math.radians(ZERO_YAW_GRIPPER)) * rotz(math.radians(g_yaw)) * roty(math.radians(g_pitch)) * rotx(math.radians(g_roll))
            gripper.setPose(gripper_pose_new)
            gripper_orientation_msg = f"R={round(g_roll)} P={round(g_pitch)} W={round((g_yaw+ZERO_YAW_GRIPPER)%360)}"     
            if s1 == 0:
                #Obre la pinça → deixa anar l’agulla
                needle.setParentStatic(base)
                status_message = "🟢 S1 premut: agulla alliberada"

            elif s1 == 1:
                #Tanca la pinça → agafa l’agulla
                needle.setParent(gripper)
                needle.setPose(TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0]))
                status_message = "🔵 S2 premut: agulla agafada"
                    
        with data_lock:
            st = Servo_torques
        if st:
            t_r1 = st.get("t_roll1", 0)
            t_r2 = st.get("t_roll2", 0)
            t_p = st.get("t_pitch", 0)
            t_y = st.get("t_yaw", 0)
            servo_torques_msg = f"T(R1,R2,P,Y) = {t_r1:.1f}, {t_r2:.1f}, {t_p:.1f}, {t_y:.1f}"
            
            #AÑADIDO
            torque_mean = abs(t_r1) + abs(t_r2) + abs(t_p) + abs(t_y)
            torque_mean /= 4.0 # Calculamos la media de los 4 torques

            #
            if torque_mean <= 0.75:
                torque_color = "green"
            elif (torque_mean <= 1.5) & (torque_mean > 0.75):
                torque_color = "yellow"
            else:
                torque_color = "red"

            # Actualizar el color del botón en la ventana principal
            torque_button.after(0, lambda: torque_button.config(bg=torque_color))

            
        # Update the label with the latest values
        update_text_label(text_label, endowrist_orientation_msg, gripper_orientation_msg, status_message, servo_torques_msg)

        time.sleep(READ_INTERVAL_S)# define the reading interval

def on_closing():
    global root, sock
    print("Closing...")
    try:
        sock.close()
        print("Ending Socket")
        initialize_robodk()
        print("Program INITIALIZED")
    except Exception as e:
        #print(f"Error al tancar el socket: {e}")
        pass
    root.destroy()
# Update functions for sliders
def set_zero_yaw_tool(value):
    global ZERO_YAW_TOOL
    ZERO_YAW_TOOL = float(value)

def set_zero_yaw_gripper(value):
    global ZERO_YAW_GRIPPER
    ZERO_YAW_GRIPPER = float(value)
# Main function
def main():
    global root, ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, robot, gripper, base, text_label
    
    robot, base, gripper, needle = initialize_robodk()

    root = tk.Tk()
    # AÑADIDO
    torque_button = tk.Button(root, text="Torque Level", width=20, height=2, bg="green", fg="white")
    torque_button.pack(pady=10) # Para mostrar el botón en la interfaz
    
    root.title("Suture Process")
    root.protocol("WM_DELETE_WINDOW", on_closing) # Proper clossing
    text_label = tk.Label(root, text="", wraplength=300)
    text_label.pack(padx=20, pady=20)

    # Add sliders for ZERO_YAW_TOOL and ZERO_YAW_GRIPPER
    tool_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Tool Yaw",
                                    command=lambda value: set_zero_yaw_tool(float(value)), length=200)
    tool_yaw_slider.set(ZERO_YAW_TOOL)
    tool_yaw_slider.pack()

    gripper_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Gripper Yaw",
                                        command=lambda value: set_zero_yaw_gripper(float(value)), length=200)
    gripper_yaw_slider.set(ZERO_YAW_GRIPPER)
    gripper_yaw_slider.pack()

    # Start the UDP reading thread
    udp_thread = threading.Thread(target=read_data_UDP)
    udp_thread.daemon = True
    udp_thread.start()

    # Start the robot movement thread
    # AÑADIDO: Hemos añadido torque_button a robot_thread
    robot_thread = threading.Thread(target=move_robot, args=(robot, gripper, needle, text_label, torque_button))
    robot_thread.daemon = True
    robot_thread.start()

    root.mainloop()

if __name__ == "__main__":
    main()