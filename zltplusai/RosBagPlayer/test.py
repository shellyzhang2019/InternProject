import pyglet
import rospy
from sensor_msgs.msg import Image as ROSImage
import argparse
import time
import threading
import multiprocessing
from PIL import Image
import io

screens = pyglet.canvas.get_display().get_screens()
windows = pyglet.window.Window(3840, 1080, fullscreen=False, screen=screens[0])

@windows.event
def show(data):
    windows.switch_to()
    windows.clear()
    windows.dispatch_events()
    img = pyglet.resource.image('highway_ramp_merge_screen_capture.png')
    img.blit(0, 0, windows.width, windows.height)
    windows.flip()


def func():
    rospy.Subscriber('/front_left_camera/image_color/compressed', ROSImage, callback=show)

if __name__ == "__main__":
    rospy.init_node('RosCameraImageView')
    thread = threading.Thread(target=func)
    thread.start()
    pyglet.app.run()