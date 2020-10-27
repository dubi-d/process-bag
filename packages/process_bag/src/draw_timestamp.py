#!/usr/bin/env python3

import rosbag
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def draw_timestamps(folder, name_in, name_out, img_topic):
    bag_in = rosbag.Bag(folder + name_in)
    bridge = CvBridge()

    with rosbag.Bag(folder + name_out, "w") as bag_out:
        for topic, msg, t in bag_in.read_messages(topics=img_topic):
            cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="mono8")
            cv_img_out = modify_img(cv_img)
            img_out = bridge.cv2_to_compressed_imgmsg(cv_img_out)
            bag_out.write(topic, img_out)

        bag_in.close()

def modify_img(img_in):
    return img_in


# Config
bag_folder = "/home/rosbags/"
bag_name = "amod20-rh3-ex-record-David_Dubach.bag"
bag_name_out = "amod20-rh3-ex-process-David_Dubach.bag"
img_topic = "/diodak/camera_node/image/compressed"

if __name__ == "__main__":
    draw_timestamps(bag_folder, bag_name, bag_name_out, img_topic)


#np_img = np.fromstring(msg.data, np.uint8)
#cv_img = cv2.imdecode(np_img, cv2.CV_LOAD_IMAGE_COLOR)
#cv_img_out = modify_img(cv_img)
#img_out = bridge.cv2_to_imgmsg(cv_img_out)
#bag_out.write(topic, img_out)