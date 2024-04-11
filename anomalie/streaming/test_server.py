import asyncio
import websockets
import cv2
import numpy as np

async def video_stream(websocket, path):
    cap = cv2.VideoCapture(0)  # Open the default webcam (you may need to change the index)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Encode the frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_data = buffer.tobytes()

            # Send the frame data to the client
            await websocket.send(jpg_data)
    except websockets.exceptions.ConnectionClosedError:
        pass
    finally:
        cap.release()

start_server = websockets.serve(video_stream, "0.0.0.0", 8765)  # You can change the port as needed

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()