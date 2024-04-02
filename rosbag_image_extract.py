# Extract Images From ROSBAG
#!/usr/bin/env python
import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    args.output_dir = os.path.join("data/", args.output_dir)
    args.bag_file = os.path.join('data/bags/', args.bag_file)

    print("Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir))
    
    output_dir_exists = os.path.exists(args.output_dir)

    if not output_dir_exists:
        os.makedirs(args.output_dir)
        print("Created directory %s" % (args.output_dir))
    else:
        print("Output directory %s already exists" % (args.output_dir))

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), cv_img)
        print("Wrote image %i" % count)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()