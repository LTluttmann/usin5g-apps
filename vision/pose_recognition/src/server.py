#!/usr/bin/env python

import asyncio
from typing import Any
import websockets
from websockets.server import WebSocketServerProtocol
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import logging
import argparse
from collections import defaultdict
from inference import detect_gesture
from stream import get_stream
from config import Config


logging.basicConfig(level=logging.DEBUG)


class EventQueue(object):
    """Use this class to count the number of frames a specific event was detected
    and to send a event message only if a specified number of consecutive frames 
    exhibit the required event
    """
    def __init__(self, min_num_frames) -> None:
        self.events = defaultdict(lambda: 0)
        self.min_num_frames = min_num_frames

    def _enqueue(self, event):
        self.events[event] += 1
        # we keep track of same event. Reset all others
        for k in self.events.keys():
            if k != event:
                self._dequeue(k)

    def _dequeue(self, event):
        self.events[event] = 0

    def __call__(self, event) -> Any:
        
        if event is not None:
            self._enqueue(event)
            if self.events[event] >= self.min_num_frames:
                self._dequeue(event)
                return event

        return None


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stream_port', type=str, default="123")
    parser.add_argument('--model_file', type=str, default="/Users/luttmann/Documents/USIN5G/Repos/usin5g-apps/vision/pose_recognition/assets/gesture_recognizer.task")
    args = parser.parse_args()
    return args


# STEP 1: Fetch the video stream
args = get_args()
config = Config(port=args.stream_port,
                model_path=args.model_file)


stream = get_stream(config)

# STEP 2: Create an GestureRecognizer object.
base_options = python.BaseOptions(model_asset_path=config.model_path)
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)


async def fetch_and_analyse_video(websocket):
    event_queue = EventQueue(5)

    try:
        
        while True: 
            in_bytes = stream.stdout.read(config.width * config.height * 3)
            if not in_bytes and stream.poll() is not None:
                logging.warning('Did not receive an image')
                break

            in_frame = np.frombuffer(in_bytes, np.uint8)
            in_frame = in_frame.reshape([config.height, config.width, 3])

            event = event_queue(detect_gesture(in_frame, recognizer))

            if event is not None:
                await websocket.send(event)

            await asyncio.sleep(0.0)

    except Exception as e:
        print(e.__str__)
        raise e


async def main(websocket: WebSocketServerProtocol, path):

    try:

        msg = await websocket.recv()
        logging.info(msg)
        await websocket.send("Hello from Server")

        video_task = asyncio.create_task(fetch_and_analyse_video(websocket))
        logging.info("Start Job for client with id: %s" % websocket.id)
        await video_task
        logging.info("Finished Job for client with id: %s" % websocket.id)

    except Exception as e:
        # TODO improve error handling
        logging.debug("Got an Error: %s" % e.__str__)
        websocket.close()
        raise e




if __name__ == "__main__":
    try:
        start_server = websockets.serve(main, "127.0.0.1", 1234, ping_timeout=120)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()

    except KeyboardInterrupt as e:

        logging.info("Terminating")
        stream.terminate()
        asyncio.get_event_loop().close()
        raise e