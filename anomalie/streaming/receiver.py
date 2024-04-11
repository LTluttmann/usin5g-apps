from flask import Flask, render_template, Response
import socket
import cv2
import pickle
import struct

app = Flask(__name__)

# Create a socket connection to Computer A



@app.route('/')
def index():
    return render_template('index.html')

def gen():

    while True:
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.bind(("0.0.0.0", 12345))
            server_socket.listen(10)
            conn, addr = server_socket.accept()
            break
        except:
            print("WHUUUT")
            continue


    data = b""
    payload_size = struct.calcsize("Q")

    while True:
        try:
            while len(data) < payload_size:
                data += conn.recv(4096)

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += conn.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = pickle.loads(frame_data)

            # Yield the frame for streaming
            print("y")
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + cv2.imencode('.jpg', frame)[1].tobytes() + b'\r\n')
        except Exception as e:
            conn.close()
            raise e

@app.route('/video_feed')
def video_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8082, debug=True)
