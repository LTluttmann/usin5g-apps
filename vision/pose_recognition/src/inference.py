import ffmpeg
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from config import Config


def get_stream(url:str):
    return (
        ffmpeg
        .input(url, framerate='15')
        .output('pipe:', format='rawvideo', pix_fmt='rgb24')
        .run_async(pipe_stdout=True)
    )


def detect_gesture(framergb, recognizer):

    rgb_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=framergb)
    recognition_result = recognizer.recognize(rgb_frame)
    gestures = recognition_result.gestures

    if len(gestures) > 0:
        gesture = gestures[0][0]
        gesture_name = gesture.category_name
        #print(gesture)
        score = gesture.score
    
        if gesture_name == "Pointing_Up" and score > 0.7:
            return "up"
        elif gesture_name == "Closed_Fist" and score > 0.7:
            return "down"
        
    return None
