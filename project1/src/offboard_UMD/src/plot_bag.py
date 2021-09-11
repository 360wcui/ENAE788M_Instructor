import os
import argparse

# import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    # parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    # parser.add_argument("bag_file", help="Input ROS bag.")
    # parser.add_argument("output_dir", help="Output directory.")
    # parser.add_argument("image_topic", help="Image topic.")
    #
    # args = parser.parse_args()
    #
    # print("Extract images from %s on topic %s into %s" % (args.bag_file,
    #                                                       args.image_topic, args.output_dir))

    bag_file = "2021-09-07-23-13-42.bag"
    bag = rosbag.Bag(bag_file, "r")
    topic = "/mavros/local_position/pose"
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), cv_img)
        print("topic" % topic)
        print("msg" % msg)
        print("Wrote image %i" % count)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()