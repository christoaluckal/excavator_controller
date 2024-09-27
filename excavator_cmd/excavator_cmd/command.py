import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16,Float32MultiArray
from deltacan.msg import DeltaCan
import time
import sys
import numpy as np
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('command_node')

        self.declare_parameter('int_number', 5)

        self.deltacan_str = '\n\
        0 mslewcmd \n\
        1 mboomcmd  \n\
        2 mbucketcmd \n\
        3 marmcmd \n\
        4 mlefttravelcmd \n\
        5 mrighttravelcmd \n\
        6 mbladecmd_a \n\
        7 mswingcmd_a \n\
        8 mthumbcmd_a \n\
        9 mbladecmd_b \n\
        10 mswingcmd_b \n\
        11 mthumbcmd_b \n\
        '

        self.key_dict = {
            0: "mslewcmd",
            1: "mboomcmd",
            2: "mbucketcmd",
            3: "marmcmd",
            4: "mlefttravelcmd",
            5: "mrighttravelcmd",
            6: "mbladecmd_a",
            7: "mswingcmd_a",
            8: "mthumbcmd_a",
            9: "mbladecmd_b",
            10: "mswingcmd_b",
            11: "mthumbcmd_b"
        }


        self.publisher_ = self.create_publisher(DeltaCan, 'tw', 10)
        self.subscriber = self.create_subscription(Float32MultiArray, 'system_id', self.callback, 10)

        print(self.deltacan_str,'\n')

    def callback(self, msg:Float32MultiArray):

        data = msg.data

        if len(data) != 3:
            print("Incorrect format")
            return
        
        cmd = int(data[0])
        value = data[1]
        duration = data[2]

        start = time.time()

        print(f"{bcolors.OKGREEN} Running [{self.key_dict[cmd]}] with power [{value}] for [{duration}] seconds. {bcolors.ENDC}")

        while True:
            if time.time() - start > duration:
                break
            print(f"\t {np.round(duration - time.time() + start,2)}",end="\r")
            deltacan_msg = DeltaCan()
            if cmd == 0:
                deltacan_msg.mslewcmd = value
            if cmd == 1:
                deltacan_msg.mboomcmd = value
            if cmd == 2:
                deltacan_msg.mbucketcmd = value
            if cmd == 3:
                deltacan_msg.marmcmd = value
            if cmd == 4:
                deltacan_msg.mlefttravelcmd = value
            if cmd == 5:
                deltacan_msg.mrighttravelcmd = value
            if cmd == 6:
                deltacan_msg.mbladecmd_a = value
            if cmd == 7:
                deltacan_msg.mswingcmd_a = value
            if cmd == 8:
                deltacan_msg.mthumbcmd_a = value
            if cmd == 9:
                deltacan_msg.mbladecmd_b = value
            if cmd == 10:
                deltacan_msg.mswingcmd_b = value
            if cmd == 11:
                deltacan_msg.mthumbcmd_b = value
            
            self.publisher_.publish(deltacan_msg)


        print(f"{bcolors.OKGREEN} Done. {bcolors.ENDC}")
        print(self.deltacan_str)
        print("-"*50)



def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()