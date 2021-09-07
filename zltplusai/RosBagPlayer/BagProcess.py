import rosbag
import argparse
from PIL import Image as PILImage
import io
from sensor_msgs.msg import Image as ROSImage
import numpy as np
import time

def process(data, resize, stamp):
    img = PILImage.open(io.BytesIO(data.data))
    img = img.resize((resize[0], resize[1]), PILImage.ANTIALIAS)
    img = img.convert('RGB')
    Img = ROSImage()
    Img.header.stamp = stamp
    Img.height = img.height
    Img.width = img.width
    Img.header.frame_id = "camera"
    Img.is_bigendian = False
    Img.encoding = 'rgb'
    Img.data = np.array(img).tobytes()
    Img.step = Img.width * 3
    return Img

def main():
    parser = argparse.ArgumentParser(description="Generate New ROS Bag")
    parser.add_argument('inputbag', type=str, nargs='+')
    parser.add_argument('-t', '--topics', type=str, nargs='+')
    parser.add_argument('-opt', '--options', type=str, nargs='+', default=(1920, 1080))
    args = parser.parse_args()
    start = time.time()
    for bag in args.inputbag:
        newbag = bag.split('.')[0] + '_ImageView.bag'
        with rosbag.Bag(newbag, 'w') as o:
            with rosbag.Bag(bag, 'r') as i:
                for topic, msg, t in i.read_messages():
                    if topic not in args.topics:
                        o.write(topic, msg, t)
                    else:
                        print('catched topic {}'.format(topic))
                        newmsg = process(msg, args.options, t)
                        o.write(topic, newmsg, t)
    end = time.time()
    print('Used time {}'.format(end - start))
    
    return 0

if __name__ == "__main__":
    main()
                        


