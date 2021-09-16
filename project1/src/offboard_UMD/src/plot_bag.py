import os
import argparse

# import cv2

import rosbag
import matplotlib.pyplot as plt
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
import numpy as np


def plot_bb2(x_positions, y_positions, z_positions, freq, filename):
    fig_no = 1
    # for symbol in symbols:
    plt.figure(fig_no)
    # filename = "x_position.jpg"
    num_samples = len(x_positions)
    total_time_elapsed = num_samples / freq
    print(total_time_elapsed, num_samples)
    seconds = np.linspace(0, total_time_elapsed, num_samples)
    plt.plot(seconds, x_positions)
    plt.plot(seconds, y_positions)
    plt.plot(seconds, z_positions)
    plt.title('Transient Analysis')
    plt.xlabel("Time(s)")
    plt.ylabel("Distance(m)")
    # plt.xlim([start_date, end_date])
    plt.xticks(rotation=30)
    # plt.plot(rolling_prices_mean[symbol])
    # plt.plot(upper_bound[symbol])
    # plt.plot(lower_bound[symbol])
    plt.legend(['x positions', 'y positions', 'z positions'])

    plt.savefig(filename)
    # plt.close()

def plot_bb(positions, freq, title, filename):
    # fig_no = 1
    # for symbol in symbols:
    # plt.figure(fig_no)
    # filename = "x_position.jpg"
    num_samples = len(positions)
    total_time_elapsed = num_samples / freq
    print(total_time_elapsed, num_samples)
    seconds = np.linspace(0, total_time_elapsed, num_samples)
    plt.plot(seconds, positions)
    plt.title(title)
    plt.xlabel("Time(s)")
    plt.ylabel("Distance(m)")
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

    # bag_file = "2021-09-07-23-13-42.bag"
    bag_file = "2021-09-12-07-22-48.bag"
    bag = rosbag.Bag(bag_file, "r")
    topic = "/mavros/local_position/pose"
    # bridge = CvBridge()
    count = 0
    x_positions = []
    y_positions = []
    z_positions = []
    for topic, msg, t in bag.read_messages(topics=[topic]):

        positions = msg.pose.position
        x_positions.append(positions.x)
        y_positions.append(positions.y)
        z_positions.append(positions.z)
    bag.close()

    plot_bb2(x_positions=x_positions, y_positions=y_positions, z_positions=z_positions, freq=20.0, filename="response.jpg")
    # plot_bb(positions=y_positions, freq=20.0, title="Y Positions", filename="y_position.jpg")
    # plot_bb(positions=z_positions, freq=20.0, title="Z Positions", filename="z_position.jpg")
    return

if __name__ == '__main__':
    main()