import os
import argparse

# import cv2

import rosbag
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge


def plot_bb(positions, title, filename):
    # fig_no = 1
    # for symbol in symbols:
    # plt.figure(fig_no)
    # filename = "x_position.jpg"
    plt.plot(positions)
    plt.title(title)
    # plt.xlim([start_date, end_date])
    plt.xticks(rotation=30)
    # plt.plot(rolling_prices_mean[symbol])
    # plt.plot(upper_bound[symbol])
    # plt.plot(lower_bound[symbol])
    # plt.legend([symbol, symbol + ' SMA', symbol + ' SMA + 2 * STD', symbol + ' SMA - 2 * STD'])
    plt.savefig(filename)
    plt.close()


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
    # bridge = CvBridge()
    count = 0
    x_positions = []
    y_positions = []
    z_positions = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), cv_img)
        # print("topic", topic)
        # print("msg", msg)
        # print("msg", msg.pose.position)
        positions = msg.pose.position
        x_positions.append(positions.x)
        y_positions.append(positions.y)
        z_positions.append(positions.z)
        # print("Wrote image %i" % count)

        # count += 1

    bag.close()
    plot_bb(x_positions, "X Positions", "x_position.jpg")
    plot_bb(y_positions, "Y Positions", "y_position.jpg")
    plot_bb(z_positions, "Z Positions", "z_position.jpg")
    return

if __name__ == '__main__':
    main()