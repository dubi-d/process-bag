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
            cv_img = bridge.compressed_imgmsg_to_cv2(msg)
            cv_img_out = modify_img(cv_img, msg.header.stamp)
            img_out = bridge.cv2_to_compressed_imgmsg(cv_img_out)
            img_out.header = msg.header
            img_out.format = msg.format
            bag_out.write(topic, img_out, t)

        bag_in.close()


def modify_img(img, timestamp):
    org = (5, 20)
    font_scale = 0.5
    color = (0, 255, 0)
    thickness = 1
    cv2.putText(img, str(timestamp), org, cv2.FONT_HERSHEY_SIMPLEX, 
                font_scale, color, thickness, cv2.LINE_AA)
    return img


# Config
bag_folder = "/home/rosbags/"
bag_name = "amod20-rh3-ex-record-David_Dubach.bag"
bag_name_out = "amod20-rh3-ex-process-David_Dubach.bag"
img_topic = "/diodak/camera_node/image/compressed"

if __name__ == "__main__":
    print("Proccessing " + bag_name)
    draw_timestamps(bag_folder, bag_name, bag_name_out, img_topic)
    print("Finished! New bagfile: " + bag_name_out)
