#!/usr/bin/env python3
import numpy as np
import customtkinter as ctk
from PIL import Image as PILImage, ImageTk
import cv2

# Set the appearance mode to light
ctk.set_appearance_mode("light")

# Global variables for video frame
video_frame = np.zeros((480, 840, 3), dtype=np.uint8)  # Black image as default

# Key event function
def key(event):
    if event.char == event.keysym:  # standard keys
        if event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
    else:  # non standard keys
        if event.keysym == 'Up':
            print("Up arrow pressed")
        elif event.keysym == 'Down':
            print("Down arrow pressed")
        elif event.keysym == 'Left':
            print("Left arrow pressed")
        elif event.keysym == 'Right':
            print("Right arrow pressed")
        elif event.keysym == 'a':  # Rotate left
            print("Yaw left (a) pressed")
        elif event.keysym == 'd':  # Rotate right
            print("Yaw right (d) pressed")

def update_video():
    if video_frame is not None:
        img = PILImage.fromarray(video_frame)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)
    root.after(30, update_video)  # Refresh rate of 30ms

# Function to start the drone (placeholder for now)
def start_drone():
    print("Start button pressed")
    # Here, you will add the drone starting logic

#---- MAIN FUNCTION
#- Create the GUI
root = ctk.CTk()
root.geometry("1024x720")
root.title("SADEN GUI")

# Create a dark frame on the right side
right_frame = ctk.CTkFrame(root, width=1024, height=720, fg_color='#222222')
right_frame.place(x=280, y=0)

# Create a frame for the video
video_frame_ctk = ctk.CTkFrame(root, width=840, height=480, fg_color='#000000')
video_frame_ctk.place(x=320, y=10)  # Centered horizontally
video_label = ctk.CTkLabel(video_frame_ctk)
video_label.pack(fill='both', expand=True)

# Add a Start button
start_button = ctk.CTkButton(root, text="Start", fg_color="#DA0037", width=300, height=100, font=("calibri", 75, 'bold'), corner_radius=15, command=start_drone, bg_color='#222222')
start_button.place(x=485, y=550)  # Centered horizontally

# Add logos image above the data
logos_img = PILImage.open(r"Assets/Logos.png")  # Update with the path to your logo
logos_img = logos_img.resize((200, 200), PILImage.ANTIALIAS)
logos_photo = ImageTk.PhotoImage(logos_img)
logos_label = ctk.CTkLabel(root, image=logos_photo, text="")
logos_label.place(x=50)

# Add labels and entry boxes for data
altitude_label = ctk.CTkLabel(root, text="Altitude", font=("calibri", 20, 'bold'))
altitude_label.place(x=25, y=150)

altitude_value = ctk.CTkLabel(root, text="N/A", width=100, height=80, fg_color="white", text_color="black", corner_radius=5)
altitude_value.place(x=10, y=190)

speed_label = ctk.CTkLabel(root, text="Speed", font=("calibri", 20, 'bold'))
speed_label.place(x=30, y=300)

speed_value = ctk.CTkLabel(root, text="N/A", width=100, height=80, fg_color="white", text_color="black", corner_radius=5)
speed_value.place(x=10, y=340)

pos_x_label = ctk.CTkLabel(root, text="Position X", font=("calibri", 20, 'bold'))
pos_x_label.place(x=160, y=150)

pos_x_value = ctk.CTkLabel(root, text="N/A", width=100, height=80, fg_color="white", text_color="black", corner_radius=5)
pos_x_value.place(x=150, y=190)

pos_y_label = ctk.CTkLabel(root, text="Position Y", font=("calibri", 20, 'bold'))
pos_y_label.place(x=160, y=300)

pos_y_value = ctk.CTkLabel(root, text="N/A", width=100, height=80, fg_color="white", text_color="black", corner_radius=5)
pos_y_value.place(x=150, y=340)

# Add another logo at the bottom left
bottom_logo_img = PILImage.open(r"Assets/Picture1.png")  # Update with the path to your logo
bottom_logo_img = bottom_logo_img.resize((300, 300), PILImage.ANTIALIAS)
bottom_logo_photo = ImageTk.PhotoImage(bottom_logo_img)
bottom_logo_label = ctk.CTkLabel(root, image=bottom_logo_photo, text="")
bottom_logo_label.place(x=10, y=450)

print(">> Control the drone with the arrow keys. Press r for RTL mode, a for yaw left, d for yaw right")
root.bind_all('<Key>', key)

# Start the video update loop
update_video()

# Start the Tkinter main loop
root.mainloop()
