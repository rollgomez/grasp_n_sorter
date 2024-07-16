#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.save_path = os.path.expanduser('~/pic2train')
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        self.image_count = self.get_initial_image_count()
        self.last_saved_time = time.time()

    def get_initial_image_count(self):
        files = os.listdir(self.save_path)
        image_files = [f for f in files if f.startswith("image_") and f.endswith(".jpg")]
        if image_files:
            image_numbers = [int(f.split('_')[1].split('.')[0]) for f in image_files]
            return max(image_numbers) + 1
        else:
            return 0

    def image_callback(self, data):
        current_time = time.time()
        if current_time - self.last_saved_time >= 10:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                image_filename = os.path.join(self.save_path, "image_{}.jpg".format(self.image_count))
                cv2.imwrite(image_filename, cv_image)
                rospy.loginfo("Saved image: %s", image_filename)
                self.image_count += 1
                self.last_saved_time = current_time
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    rospy.init_node('image_saver_node', anonymous=True)
    ImageSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")