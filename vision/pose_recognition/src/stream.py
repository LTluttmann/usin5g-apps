import subprocess
import threading
import time
import logging
from config import Config


logging.basicConfig(level = logging.INFO)


class CaptureStreamException(Exception):
    "Raised when the stream frames cannot be captured"
    pass


def connect(command, config):
    process = subprocess.Popen(command, stdout=subprocess.PIPE)
    # wait until stream is ready to fire up server
    _ = process.stdout.read(config.width * config.height * config.channels)
    return process
    

def connect_with_timeout(command, config: Config):
    process = subprocess.Popen(command, stdout=subprocess.PIPE)

    def target():
        _ = process.stdout.read(config.width * config.height * config.channels)

    thread = threading.Thread(target=target)
    thread.start()
    thread.join(config.stream_connect_timeout)

    if thread.is_alive():
        # The process is still running; terminate it
        process.terminate()
        thread.join()
        raise subprocess.TimeoutExpired(command, config.stream_connect_timeout)

    return process


def get_stream(config: Config):

    cmd = [
        'ffmpeg', 
        "-fflags", "nobuffer",
        # '-r', '{}'.format(config.input_rate), 
        '-i', 'udp://localhost:{}'.format(config.stream_port), 
        "-probesize", "32", 
        "-analyzeduration",  "0",
        '-f', 'rawvideo', 
        '-pix_fmt', 'rgb24', 
        'pipe:'
    ]

    max_attempts = config.stream_connect_attempts
    num_attempts = 0
    stream = None

    while stream is None :
        try:
            num_attempts += 1
            if config.stream_connect_timeout is not None:
                stream = connect_with_timeout(cmd, config)
            else:
                stream = connect(cmd, config)
        except subprocess.TimeoutExpired as e:
            if num_attempts <= max_attempts:
                logging.info("Connection failed (%s attempts left)" % (max_attempts - num_attempts))
                time.sleep(10.0)
            else:
                raise e
            
    return stream

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np
    config = Config(stream_port=123)
    stream = get_stream(config)
    in_bytes = stream.stdout.read(config.width * config.height * config.channels)
    in_frame = np.frombuffer(in_bytes, np.uint8)
    in_frame = in_frame.reshape([config.height, config.width, config.channels])
    plt.imshow(in_frame)
