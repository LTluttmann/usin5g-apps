#!/usr/bin/env python

import asyncio
import websockets
from websockets.server import WebSocketServerProtocol
from inference import get_stream, detect_gesture
from config import Config
import argparse
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import logging
import time


logging.basicConfig(level=logging.DEBUG)


class EventQueue(object):
    """Use this class to count the number of frames a specific event was detected
    and to send a event message only if a specified number of consecutive frames 
    exhibit the required event
    """
    def __init__(self, min_num_frames) -> None:
        pass

    def _enqueue(self, event):
        ...

    def _dequeue(self):
        ...


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stream_port', type=str, default="2345")
    parser.add_argument('--model_file', type=str, default="/Users/luttmann/Documents/USIN5G/Repos/usin5g-apps/vision/pose_recognition/assets/gesture_recognizer.task")
    args = parser.parse_args()
    return args


# STEP 1: Fetch the video stream
args = get_args()
config = Config(model_path=args.model_file)

while True:
    try:
        stream = get_stream("udp://127.0.0.1:%s" % args.stream_port)
        break
    except Exception as e:
        print(e.__str__)
        time.sleep(5.0)


# STEP 2: Create an GestureRecognizer object.
base_options = python.BaseOptions(model_asset_path=config.model_path)
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)


async def fetch_and_analyse_video(websocket):
    while True: 
        in_bytes = stream.stdout.read(config.width * config.height * 3)
        if not in_bytes and stream.poll() is not None:
            break

        in_frame = np.frombuffer(in_bytes, np.uint8)
        in_frame = in_frame.reshape([config.height, config.width, 3])

        event = detect_gesture(in_frame, recognizer)

        if event is not None:
            await websocket.send(event)

        await asyncio.sleep(0.0)


async def main(websocket: WebSocketServerProtocol, path):

    try:

        msg = await websocket.recv()
        print(msg)
        await websocket.send("Hello from Server")

        video_task = asyncio.create_task(fetch_and_analyse_video(websocket))

        await video_task

    except:
        print("Closing video feed connection")


if __name__ == "__main__":
    start_server = websockets.serve(main, "127.0.0.1", 1234, ping_timeout=120)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()