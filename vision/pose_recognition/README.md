start stream with ffmpeg, e.g.

```
ffmpeg -f avfoundation -r 30 -s "1280x720" -i "0" -vf scale=480:-1 -r 10 -f mpegts udp://192.168.178.27:12345
```

ffmpeg -f avfoundation -r 30 -s "1280x720" -i "0" -preset ultrafast -vcodec libx264 -tune zerolatency -vf scale=480:-1 -b 900k -f h264 udp://192.168.178.27:12345