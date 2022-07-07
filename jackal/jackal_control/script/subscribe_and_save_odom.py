#!/usr/bin/env python
import rospy
import argparse
import tf
import csv
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

COUNT = 0
SAVE_PATH = ""
WRITE_MODE = "XYYaw"
TIME_CRITERIA = ""

def OdomCallback(OdomMsg):
    global COUNT
    COUNT += 1
    print(str(COUNT) + "-th message is comming!")
    x = OdomMsg.pose.pose.position.x
    y = OdomMsg.pose.pose.position.y
    z = OdomMsg.pose.pose.position.z
    quaternion = (
        OdomMsg.pose.pose.orientation.x,
        OdomMsg.pose.pose.orientation.y,
        OdomMsg.pose.pose.orientation.z,
        OdomMsg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    timestamp = OdomMsg.header.stamp #.nsecs * 1e-9
    

    with open(SAVE_PATH, 'a') as file:
        writer = csv.writer(file, lineterminator="\n")
        writer.writerow([timestamp, x, y, yaw])

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/odometry/filtered", Odometry, OdomCallback)

    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Save odom message to csv file")
    parser.add_argument('--file_path', '-f', default="", type=str, help="Absolute path of the result file")
    parser.add_argument('--write_mode', '-w', default="XYYaw", type=str, help="Style of writing poses")
    parser.add_argument('--write_header', '-d', default=True, type=bool, help="Whether write header or not")
    parser.add_argument('--time_criteria', '-t', default="absolute", type=str, help="The initial time criteria")

    args = parser.parse_args()
    print("Start subscriber!")
    print("\033[1;32mSave path: " +  args.file_path + "\033[0m")
    SAVE_PATH = args.file_path
    WRITE_MODE = args.write_mode
    TIME_CRITERIA = args.time_criteria

    with open(SAVE_PATH, 'w') as file:
        writer = csv.writer(file, lineterminator="\n")
        if (args.write_header):
            if (WRITE_MODE == "XYYaw"):
                writer.writerow(['timestamp', 'x', 'y', 'yaw'])

    listener()