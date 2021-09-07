import pyglet
import rospy
from sensor_msgs.msg import CompressedImage as ROSImage
import argparse
from PIL import Image as PILImage
import numpy as np
import threading
import sys
import time
import io
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def ImageProcess(data, resize):
    cv_img = bridge.compressed_imgmsg_to_cv2(data, "rgb8")
    cv_img = cv2.resize(cv_img, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
    img = np.asarray(cv_img).tobytes()
    return img

class Player:
    
    def __init__(self, topics, screens, fullscreen=False, default_size=(1920, 1080)) -> None:
        self.window_1 = pyglet.window.Window(default_size[0], default_size[1], fullscreen=fullscreen, screen=screens[0])
        self.window_2 = pyglet.window.Window(default_size[0], default_size[1], fullscreen=fullscreen, screen=screens[1])
        # self.window_3 = pyglet.window.Window(default_size[0], default_size[1], fullscreen=fullscreen, screen=screens[2])
        # self.window_4 = pyglet.window.Window(default_size[0], default_size[1], fullscreen=fullscreen, screen=screens[3])
        self.msgs = {}
        self.windows = {}
        for topic in topics:
            self.msgs[topic] = None
            if topic == '/front_left_camera/image_color/compressed':
                self.windows[topic] = self.window_1
            elif topic == '/front_right_camera/image_color/compressed':
                self.windows[topic] = self.window_2
            # elif topic == '/side_left_camera/image_color/compressed':
            #     self.windows[topic] = self.window_3
            # elif topic == '/side_right_camera/image_color/compressed':
            #     self.windows[topic] = self.window_4
        self.current_topic = None


        

    def store(self, data, topic):
        self.msgs[topic] = data
        self.current_topic = topic

    def run(self):
        for m in self.msgs:
            if self.msgs[m]:
                img = pyglet.image.ImageData(1920, 1080, "RGB", ImageProcess(self.msgs[m], (1920, 1080)), pitch = -1920 * 3)
                if m == '/front_left_camera/image_color/compressed':
                    self.window_1.switch_to()
                    self.window_1.dispatch_events()
                    self.window_1.clear()
                    img.blit(0, 0, width=1920, height=1080)
                    self.window_1.flip()
                elif m == '/front_right_camera/image_color/compressed':
                    self.window_2.switch_to()
                    self.window_2.dispatch_events()
                    self.window_2.clear()
                    img.blit(0, 0, width=1920, height=1080)
                    self.window_2.flip()
                # elif m == '/side_left_camera/image_color/compressed':
                #     self.window_3.switch_to()
                #     self.window_3.dispatch_events()
                #     self.window_3.clear()
                #     img.blit(0, 0, width=1920, height=1080)
                #     self.window_3.flip()
                # elif m == '/side_right_camera/image_color/compressed':
                #     self.window_4.switch_to()
                #     self.window_4.dispatch_events()
                #     self.window_4.clear()
                #     img.blit(0, 0, width=1920, height=1080)
                #     self.window_4.flip()
        
            
                
    def get_tag(self):
        return self.tag
    
class ImageView:

    def __init__(self, topics) -> None:
        self.display = pyglet.canvas.get_display()
        self.screens = self.display.get_screens()
        self.player = Player(topics, self.screens)

           
def main():
    rospy.init_node('RosCameraImageView')
    parser = argparse.ArgumentParser(description="RosImageView")
    parser.add_argument("--Topics", type=str, nargs='+')
    args = parser.parse_args()
    image_view = ImageView(args.Topics)
    while True:
        start_time = time.time()
        rospy.Subscriber(args.Topics[0], ROSImage, callback=image_view.player.store, callback_args=(args.Topics[0]), queue_size=1)
        rospy.Subscriber(args.Topics[1], ROSImage, callback=image_view.player.store, callback_args=(args.Topics[1]), queue_size=1)
        # rospy.Subscriber(args.Topics[2], ROSImage, callback=image_view.player.store, callback_args=(args.Topics[2]), queue_size=1)
        # rospy.Subscriber(args.Topics[3], ROSImage, callback=image_view.player.store, callback_args=(args.Topics[3]), queue_size=1)
        image_view.player.run()
        end_time = time.time()
        print(end_time - start_time)
    
    

if __name__ == "__main__":
    main()
    
    
    
        
