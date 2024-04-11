import cv2
import socket
import pickle
import struct
import time

# Initialize webcam capture and socket
cap = cv2.VideoCapture(0)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while True:
    try:
        client_socket.connect(('0.0.0.0', 12345))  # Replace with the Azure VM's IP and desired port
        print("Connected")
        break
    except:
        time.sleep(30.0)

while True:
    try:
        # Capture a frame
        ret, frame = cap.read()
        frame = cv2.resize(frame, (480, 270))
        # Serialize and send the frame
        data = pickle.dumps(frame)
        client_socket.sendall(struct.pack("L", len(data)) + data)
        print("send")
    except Exception as e:
        break

# Close the socket and release the webcam
client_socket.close()
cap.release()
raise e