#!/usr/bin/env python

import tty, termios, time
import sys, select, os
import rospy

settings = termios.tcgetattr(sys.stdin)

def GetKey():
    key = ""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    rospy.init_node('vel_publisher')
    lfile = "/dev/rtmotor_raw_l0"
    rfile = "/dev/rtmotor_raw_r0"
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            key = GetKey()
            if(key=="w"):
                with open(lfile, "w") as lf, open(rfile, "w") as rf:
                    lf.write(str(990)+'\n')
                    rf.write(str(990)+'\n')
                    print("go")
            if(key=="s"):
                with open(lfile, "w") as lf, open(rfile, "w") as rf:
                    lf.write(str(-990)+'\n')
                    rf.write(str(-990)+'\n')
                    print("back")

            if(key=="a"):
                with open(lfile, "w") as lf, open(rfile, "w") as rf:
                    lf.write(str(-409)+'\n')
                    rf.write(str(409)+'\n')
                    print("left")

            if(key=="d"):
                with open(lfile, "w") as lf, open(rfile, "w") as rf:
                    lf.write(str(409)+'\n')
                    rf.write(str(-409)+'\n')
                    print("right")

            if(key==""):
                with open(lfile, "w") as lf, open(rfile, "w") as rf:
                    lf.write(str(0)+'\n')
                    rf.write(str(0)+'\n')
                    print("stop")
            else:
                if (key == '\x03'):
                    break
    except:
        pass
    rate.sleep()
