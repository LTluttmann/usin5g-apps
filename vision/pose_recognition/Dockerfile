FROM ubuntu:20.04

WORKDIR /io

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        gcc-8 g++-8 \
        ca-certificates \
        curl \
        ffmpeg \
        git \
        wget \
        unzip \
        nodejs \
        npm \
        python3-dev \
        python3-opencv \
        python3-pip \
        libopencv-core-dev \
        libopencv-highgui-dev \
        libopencv-imgproc-dev \
        libopencv-video-dev \
        libopencv-calib3d-dev \
        libopencv-features2d-dev \
        software-properties-common && \
    add-apt-repository -y ppa:openjdk-r/ppa && \
    apt-get update && apt-get install -y openjdk-8-jdk && \
    apt-get install -y mesa-common-dev libegl1-mesa-dev libgles2-mesa-dev && \
    apt-get install -y mesa-utils && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 100 --slave /usr/bin/g++ g++ /usr/bin/g++-8
RUN pip3 install --upgrade pip
RUN pip3 install --upgrade setuptools wheel


RUN ln -s /usr/bin/python3 /usr/bin/python

EXPOSE 12345
EXPOSE 1234

COPY src /src/
COPY requirements.txt ./requirements.txt
COPY assets/gesture_recognizer.task src/gesture_recognizer.task

RUN pip3 install -r requirements.txt
RUN chmod +x /src/server.py

# entrypoint (this part should not be changed / not configurable)
ENTRYPOINT [ "python3", "/src/server.py" ]
CMD [ "--model_file", "/src/gesture_recognizer.task", "--stream_port", "12345" ]