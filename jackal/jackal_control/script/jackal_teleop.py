from __future__ import print_function


from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from select import select

import threading
import rospy
import signal
import sys
import keyboard

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

W_KEY = 119
S_KEY = 115
A_KEY = 97
D_KEY = 100
Q_KEY = 113
SPACE_KEY = 32

TwistMsg = Twist

msg = """
\033[1;32m* 'w'/'s' keys : Update linear velocity incre/decre.
* 'a'/'d' keys : Update angular velocity incre/decre.
* Space Bar : Reset linear/angular velocities.
* 'q' : Quit.\033[0m
"""

TRANS_VEL = 0
ROT_VEL = 0

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

def vels(speed, turn):
    return "current:\tTrans. speed %s\tRot. speed %s " % (speed, turn)

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.yaw = 0.0

        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, trans_vel, rot_vel):
        self.x = trans_vel
        self.yaw = rot_vel

        # Notify publish thread that we have a new message.
        # self.condition.notify()
        # self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.yaw

            self.condition.release()

            # Publish.
            # print(twist.linear.x, " / ", twist.angular.z)
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def cutoff_velocity(value, max_abs_value):
    cutoff_value = value
    if (value > max_abs_value):
        cutoff_value = max_abs_value
    elif (value < -max_abs_value):
        cutoff_value = -max_abs_value
    return cutoff_value

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    trans_vel_incre = rospy.get_param("~trans_vel_incre", 0.1)
    trans_vel_max = rospy.get_param("~trans_vel_max", 1.5)
    rot_vel_incre = rospy.get_param("~rot_vel_incre", 0.02)
    rot_vel_max = rospy.get_param("~rot_vel_max", 1.0)

    repeat = rospy.get_param("~repeat_rate", 10.0)
    key_timeout = rospy.get_param("~key_timeout", 0.1)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    TRANS_VEL = 0
    ROT_VEL = 0

    print(msg)

    try:
        # pub_thread.wait_for_subscribers()
        pub_thread.update(TRANS_VEL, ROT_VEL)
        print("HEY?")
        print(msg)
        print(vels(TRANS_VEL, ROT_VEL))
        while(1):
            signal.signal(signal.SIGINT, signal_handler)
            key = getKey(settings, key_timeout)
            ts_vel_incremental = 0.0
            angular_vel_incremental = 0.0
            if (len(key) != 0):
                if ord(key) == W_KEY:
                    print("\033[1;32m 'w' key is pressed \033[0m")
                    TRANS_VEL += trans_vel_incre
                elif ord(key) == S_KEY:
                    print("\033[1;32m 's' key is pressed \033[0m")
                    TRANS_VEL -= trans_vel_incre
                elif ord(key) == A_KEY:
                    print("\033[1;32m 'a' key is pressed \033[0m")
                    ROT_VEL += rot_vel_incre
                elif ord(key) == D_KEY:
                    print("\033[1;32m 'd' key is pressed \033[0m")
                    ROT_VEL -= rot_vel_incre
                elif ord(key) == SPACE_KEY:
                    print("Stop!")
                    TRANS_VEL = 0
                    ROT_VEL = 0
                elif ord(key) == Q_KEY:
                    print("Exit!")
                    break

                # Cut off values for the safety issue
                TRANS_VEL = cutoff_velocity(TRANS_VEL, trans_vel_max)
                ROT_VEL = cutoff_velocity(ROT_VEL, rot_vel_max)
                print(vels(TRANS_VEL, ROT_VEL))
            pub_thread.update(TRANS_VEL, ROT_VEL)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)