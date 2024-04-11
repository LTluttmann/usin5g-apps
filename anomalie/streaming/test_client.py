import asyncio
import websockets
import cv2
import numpy as np


async def receive_frames():
    async with websockets.connect('ws://0.0.0.0:8765') as websocket:  # Change the IP address and port
        while True:
            jpg_data = await websocket.recv()
            frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), 1)
            cv2.imshow('Client', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(receive_frames())
    cv2.destroyAllWindows()