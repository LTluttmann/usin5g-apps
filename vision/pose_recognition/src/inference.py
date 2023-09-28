import ffmpeg
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from config import Config

gesture_event_map = {
    "Pointing_Up": "up",
    "Closed_Fist": "down"
}


def detect_gesture(framergb, recognizer, config: Config):

    rgb_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=framergb)
    recognition_result = recognizer.recognize(rgb_frame)
    result = recognition_result.gestures

    if len(result) > 0:
        gesture = result[0][0]
        name = gesture.category_name
        #print(gesture)
        score = gesture.score

        if score > config.gesture_min_confidence:
            return gesture_event_map.get(name, None)

    return None
