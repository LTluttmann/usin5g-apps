#!/usr/bin/env python

import asyncio
import websockets
import logging


logging.basicConfig(level=logging.DEBUG)
    

class Publisher(object):
    """implement ROS logic here"""
    def __init__(self) -> None:
        self.last_msg = None

    def publish(self, msg):
        
        if msg != self.last_msg:
            self.last_msg = msg
            print("EVENT: ", msg)
            


async def message():

    publisher = Publisher()
    retry_counts = 0
    while True:
        try:
            async with websockets.connect("ws://127.0.0.1:1234") as socket:
                await socket.send("Hello from Client")
                await socket.recv()
                print("Connection esablished")

                retry_counts = 0
                while True:
                    logging.info("awaiting event message from server")
                    response = await socket.recv()
                    logging.info("received event message from server")
                    if response in ["up", "down"]:
                        publisher.publish(response)
                    elif response == "pong":
                        print("Received pong from server.")
                    else:
                        print(f"Received unexpected response from server: {response}")

        except Exception as e:
            print(e.__str__)
            print("Connection closed. Reconnecting... (%s more attempts)" % (3 - retry_counts))
            retry_counts += 1


        finally:
            if retry_counts > 3:
                print("Connection retries exeeded. Aborting...")
                break

        await asyncio.sleep(30)  # Adjust the reconnection interval as needed


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(message())