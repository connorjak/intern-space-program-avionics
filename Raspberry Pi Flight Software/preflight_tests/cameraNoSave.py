"""Display video on monitor without saving it."""

from  picamera import PiCamera
from time import sleep


try:
    camera = PiCamera()
    camera.resolution = (1024, 768)
    camera.start_preview(fullscreen=False, window=(0,0,720, 525))


    while True:
        sleep(1)

except KeyboardInterrupt:
    camera.close()
