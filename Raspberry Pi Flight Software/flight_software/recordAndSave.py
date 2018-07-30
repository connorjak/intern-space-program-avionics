"""Display video on monitor without saving it."""

from picamera import PiCamera
from time import sleep
import datetime
import os
stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
directory = 'recordedFootage/session%s' %  stamp
file = "flight" + stamp + ".h264"
os.makedirs(directory)
i = 0
camera = PiCamera()
try:
    camera.resolution = (1024, 768)
    camera.start_preview(fullscreen=False, window=(0,0,720,525))
    camera.start_recording('%s/0.h264' % directory)
    while True:
        i = i+1
        camera.wait_recording(10)
        camera.split_recording('%s/%d.h264' % (directory, i))
except KeyboardInterrupt:
    camera.stop_recording()
    camera.close()
