#!/usr/bin/env python3
import numpy as np
import customtkinter as ctk
from PIL import Image as PILImage, ImageTk
import cv2
import time
from dronekit import connect, VehicleMode
from station_comm.msg import Rangefinder
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pymavlink import mavutil
import rospy
import threading

# Set the appearance mode to light
ctk.set_appearance_mode("light")

# Global variables for video frame and altitude
video_frame = None  # Will hold the latest video frame
v_alt = 0.0
bridge = CvBridge()  # CvBridge to convert ROS Image to OpenCV

# ROS node initialization
rospy.init_node('drone_gui', anonymous=True)

# Initialize vehicle connection
print('Connecting...')
vehicle = connect('udp:192.168.1.2:14569')

# Setup the commanded flying speed
gnd_speed = 0.1  # [m/s]

current_point = np.array([0.0, 0.0, 0.0])

# Define arm and takeoff
def arm_and_takeoff(altitude):
    print(altitude)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 0.1:
            print("Target altitude reached")
            break
        time.sleep(1)
    
    # Print control instructions and bind key events after reaching target altitude
    print(">> Control the drone with the arrow keys. Press r for RTL mode, a for yaw left, d for yaw right")
    root.bind_all('<Key>', key)

# Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # BITMASK -> Consider only the velocities
        0, 0, 0,        # POSITION
        vx, vy, vz,     # VELOCITY
        0, 0, 0,        # ACCELERATIONS
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Key event function
def key(event):
    if event.char == event.keysym:  # standard keys
        if event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")
        if event.keysym == 'l':
            print("l pressed >> Set the vehicle to LAND")
            vehicle.mode = VehicleMode("LAND")
    else:  # non standard keys
        if event.keysym == 'Up':
            set_velocity_body(vehicle, gnd_speed, 0, 0)
            print("Going forward")
        elif event.keysym == 'Down':
            set_velocity_body(vehicle, -gnd_speed, 0, 0)
            print("Going backward")
        elif event.keysym == 'Left':
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
            print("Going left")
        elif event.keysym == 'Right':
            set_velocity_body(vehicle, 0, gnd_speed, 0)
            print("Going right")
        elif event.keysym == 'a':  # Rotate left
            print("Yaw left (a) pressed")
        elif event.keysym == 'd':  # Rotate right
            print("Yaw right (d) pressed")

# Function to update altitude from ROS topic
def update_altitude(msg):
    global v_alt
    v_alt = msg.distance
    #print(f"Received altitude: {v_alt}")  # Debugging print statement

# ROS Subscriber for altitude
rospy.Subscriber("/drone2/pixhawk/rangefinder", Rangefinder, update_altitude)

def set_current_pose(data):
    global current_point, v_alt

    current_point = np.array([data.pose.pose.position.x, data.pose.pose.position.y, v_alt])

rospy.Subscriber("/drone2/orbslam/filtered_odom", Odometry, set_current_pose)

# Function to update GUI elements periodically
def update_gui():
    altitude_value.configure(text=f"{v_alt:.2f} m")
    speed_value.configure(text= f"{gnd_speed:.2f} m/s")
    pos_x_value.configure(text= f"{current_point[0]:.2f} m")
    pos_y_value.configure(text= f"{current_point[1]:.2f} m")
    root.after(100, update_gui)  # Update the GUI every 100 ms

# Function to update the video frame from ROS topic
def update_video_frame(msg):
    global video_frame
    try:
        encoding = msg.encoding
        if encoding == "8UC3":
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # Convert to RGB
        else:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # Convert to RGB
        video_frame = cv_image
        #print("Video frame updated")  # Debugging print statement
    except CvBridgeError as e:
        print(f"Error converting image: {e}")

# ROS Subscriber for video stream
def ros_image_callback(msg):
    #print("Received image message")  # Debugging print statement
    update_video_frame(msg)

rospy.Subscriber("/drone2/zed/img", Image, ros_image_callback)

def update_video():
    if video_frame is not None:
        img = PILImage.fromarray(video_frame)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)
        #print("Video label updated")  # Debugging print statement
    else:
        print("No video frame to update")  # Debugging print statement
    root.after(30, update_video)  # Refresh rate of 30ms

# Function to start the drone in a separate thread
def start_drone():
    print("Start button pressed")
    threading.Thread(target=arm_and_takeoff, args=(1,)).start()

# Create the GUI
root = ctk.CTk()
root.geometry("1024x720")
root.title("SADEN GUI")

# Create a dark frame on the right side
right_frame = ctk.CTkFrame(root, width=1024, height=720, fg_color='#222222')
right_frame.place(x=350, y=0)

# Create a frame for the video
video_frame_ctk = ctk.CTkFrame(root, width=840, height=480, fg_color='#000000')
video_frame_ctk.place(x=350, y=10)  # Centered horizontally
video_label = ctk.CTkLabel(video_frame_ctk)
video_label.pack(fill='both', expand=True)

# Add a Start button
start_button = ctk.CTkButton(root, text="Start", fg_color="#DA0037", width=300, height=100, font=("calibri", 75, 'bold'), corner_radius=15, command=start_drone, bg_color='#222222')
start_button.place(x=485, y=550)  # Centered horizontally

# Add logos image above the data
logos_img = PILImage.open(r"Assets/Logos.png")
logos_img = logos_img.resize((200, 200), PILImage.Resampling.LANCZOS)
logos_photo = ImageTk.PhotoImage(logos_img)
logos_label = ctk.CTkLabel(root, image=logos_photo, text="")
logos_label.place(x=50)

# Add labels and entry boxes for data
altitude_label = ctk.CTkLabel(root, text="Altitude", font=("calibri", 20, 'bold'))
altitude_label.place(x=25, y=160)

altitude_value = ctk.CTkLabel(root, text="N/A", width=100, height=80, fg_color="white", text_color="black", corner_radius=5, font=("calibri", 17, 'bold'))
altitude_value.place(x=20, y=200)

speed_label = ctk.CTkLabel(root, text="Speed", font=("calibri", 20, 'bold'))
speed_label.place(x=30, y=310)

speed_value = ctk.CTkLabel(root, text="N/A", width=100, height=80, fg_color="white", text_color="black", corner_radius=5, font = ("calibri", 17, 'bold'))
speed_value.place(x=20, y=350)

pos_x_label = ctk.CTkLabel(root, text="Position X", font=("calibri", 20, 'bold'))
pos_x_label.place(x=180, y=160)

pos_x_value = ctk.CTkLabel(root, text="N/A", width=100, height=80, fg_color="white", text_color="black", corner_radius=5, font=("calibri", 17, 'bold'))
pos_x_value.place(x=180, y=200)

pos_y_label = ctk.CTkLabel(root, text="Position Y", font=("calibri", 20, 'bold'))
pos_y_label.place(x=180, y=310)

pos_y_value = ctk.CTkLabel(root, text="N/A", width=100, height=80, fg_color="white", text_color="black", corner_radius=5, font=("calibri", 17, 'bold'))
pos_y_value.place(x=180, y=350)

# Add another logo at the bottom left
bottom_logo_img = PILImage.open(r"Assets/Picture1.png")  # Update with the path to your logo
bottom_logo_img = bottom_logo_img.resize((300, 300), PILImage.Resampling.LANCZOS)
bottom_logo_photo = ImageTk.PhotoImage(bottom_logo_img)
bottom_logo_label = ctk.CTkLabel(root, image=bottom_logo_photo, text="")
bottom_logo_label.place(x=10, y=450)

# Start the video update loop
update_video()

# Start the GUI update loop
update_gui()

# Start the Tkinter main loop
root.mainloop()

# Keep the ROS node running
rospy.spin()
