import rospy
from sensor_msgs.msg import Image as sensorImage
from PIL import Image
import numpy as np
import cv2
class ImageHandler:
    def __init__(self, color_topic, depth_topic) -> None:
        self.color_topic = color_topic
        self.depth_topic = depth_topic

    def get_data(self):
        img_msg : sensorImage = rospy.wait_for_message(self.color_topic, sensorImage)
        depth_msg : sensorImage = rospy.wait_for_message(self.depth_topic, sensorImage)
        cv_depth = np.frombuffer(depth_msg.data, dtype=np.uint16)
        cv_depth = cv_depth.reshape(depth_msg.height, depth_msg.width)
        cv_img = np.frombuffer(img_msg.data, dtype=np.uint8)
        cv_img = cv_img.reshape(img_msg.height, img_msg.width, -1)

        pil_color = Image.fromarray(cv_img)
        pil_depth = Image.fromarray(cv_depth)
        return pil_color, pil_depth
       