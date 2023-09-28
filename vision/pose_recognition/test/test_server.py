import websockets
from websockets.server import WebSocketServerProtocol
import asyncio
import numpy as np


async def main(websocket: WebSocketServerProtocol, path):
    while True:
        try:
            if np.random.random() < .5:
                await websocket.send("up")
            else:
                await websocket.send("down")
            await asyncio.sleep(5)
        except websockets.ConnectionClosedError:
            print("Connection closed")
            break



if __name__ == "__main__":
    start_server = websockets.serve(main, "127.0.0.1", 1234)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
