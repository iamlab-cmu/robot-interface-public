#!/usr/bin/env python

import roslib
roslib.load_manifest('franka_action_lib')

import argparse
import sys
import os
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

TOPICS_LIST = ['/camera1/color/image_raw', '/camera2/color/image_raw',
               '/camera3/color/image_raw']

class ImageConverter:
    def __init__(self, topic_list):
        self.bridge = CvBridge()
        self.images_by_topic = {}

        self.image_sub_list = []
        for i, topic in enumerate(topic_list):
            cb = self.get_callback_for_topic(topic)
            image_sub = rospy.Subscriber(topic, Image, cb)
            self.image_sub_list.append(image_sub)
            print("Subscribed to topic: {}".format(topic))

    def get_callback_for_topic(self, topic):
        self.images_by_topic[topic] = []

        def callback(data):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            (rows,cols,channels) = cv_image.shape
            self.images_by_topic[topic].append(cv_image)

            if len(self.images_by_topic[topic]) % 100 == 0:
                print("Topic: {} Extracted {} images".format(
                    topic, len(self.images_by_topic[topic])))

        return callback

    def save_images_as_jpeg(self, target_dir):
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)
        print("Will save images to {}".format(target_dir))
        for topic, images in self.images_by_topic.items():
            dir_name = '_'.join(topic.split('/')[1:-1])
            dir_path = os.path.join(target_dir, dir_name)
            if not os.path.exists(dir_path):
                os.makedirs(dir_path)
            for i, img in enumerate(images):
                img_name = 'frame_{:05}.jpg'.format(i)
                img_path = os.path.join(dir_path, img_name)
                cv2.imwrite(img_path, img)
            print("Did save {} images to {}".format(len(images), dir_path))

    def save_images_to_h5(self):
        raise ValueError("To be implemented")

def main(args):
    ic = ImageConverter(TOPICS_LIST)
    any_topic = ic.images_by_topic.keys()[0]
    try:
        while not rospy.is_shutdown():
            prev_num_images = len(ic.images_by_topic[any_topic])
            # Sleep for 2s
            rospy.sleep(5)
            new_num_images = len(ic.images_by_topic[any_topic])

            if new_num_images == prev_num_images:
                print("No new images added to topic: {}. Will save and exit!".format(
                    any_topic))
                ic.save_images_as_jpeg('/home/mohit/temp_images')
                break
            else:
                print("Still adding images")

    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('extract_images_from_rosbag')

    parser = argparse.ArgumentParser(description="Extract images from rosbag")
    args = parser.parse_args()
    main(args)
