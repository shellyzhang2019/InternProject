from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import rospy
from sensor_msgs.msg import CompressedImage as ROSImage
from PIL import Image, ImageQt
import io


def window(topic):
    app = QtWidgets.QApplication(sys.argv)

    w = QtWidgets.QWidget()
    w.setGeometry(100, 100, 960, 540)
    l = QtWidgets.QLabel(w)
    while True:
        w.show()
        data = rospy.wait_for_message(topic, ROSImage)
        img = Image.open(io.BytesIO(data.data))
        # img = img.convert('RGB')
        img = img.toqpixmap()
        l.setScaledContents(True)
        l.setPixmap(img)
        print('ok')
        app.exit(app.exec_())

if __name__== "__main__":
    rospy.init_node('RosCameraImageView')
    window('/front_left_camera/image_color/compressed')