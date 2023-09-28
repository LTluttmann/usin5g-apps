import websockets
from websockets.server import WebSocketServerProtocol
import asyncio


async def main(websocket: WebSocketServerProtocol, path):
    while True:
        try:
            # input must be concurrent, otherwise the server will fail to send
            # pings to the client if the user is idle for a moment.
            msg = await input("type a message")
            await websocket.send(msg)
            await asyncio.sleep(0.5)
        except websockets.ConnectionClosedError:
            print("Connection closed")
            break



if __name__ == "__main__":
    start_server = websockets.serve(main, "0.0.0.0", 1234)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
