#! /usr/bin/env python
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

import math
import time

def map_keyboard(side):
    limb = intera_interface.Limb(side)

    try:
        gripper = intera_interface.Gripper(side + '_gripper')
    except:
        has_gripper = False
        rospy.loginfo("The electric gripper is not detected on the robot.")
    else:
        has_gripper = True

    joints = limb.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_pos = current_position + delta
        return joint_pos
        # print("Executing" + str(joint_command))
        # limb.set_joint_position_speed(0.3)
        # limb.set_joint_positions(joint_command)

    # bindings = {
    #     '1': (set_j, [limb, joints[0], 0.1], joints[0]+" increase"),
    #     'q': (set_j, [limb, joints[0], -0.1], joints[0]+" decrease"),
    #     '2': (set_j, [limb, joints[1], 0.1], joints[1]+" increase"),
    #     'w': (set_j, [limb, joints[1], -0.1], joints[1]+" decrease"),
    #     '3': (set_j, [limb, joints[2], 0.1], joints[2]+" increase"),
    #     'e': (set_j, [limb, joints[2], -0.1], joints[2]+" decrease"),
    #     '4': (set_j, [limb, joints[3], 0.1], joints[3]+" increase"),
    #     'r': (set_j, [limb, joints[3], -0.1], joints[3]+" decrease"),
    #     '5': (set_j, [limb, joints[4], 0.1], joints[4]+" increase"),
    #     't': (set_j, [limb, joints[4], -0.1], joints[4]+" decrease"),
    #     '6': (set_j, [limb, joints[5], 0.1], joints[5]+" increase"),
    #     'y': (set_j, [limb, joints[5], -0.1], joints[5]+" decrease"),
    #     '7': (set_j, [limb, joints[6], 0.1], joints[6]+" increase"),
    #     'u': (set_j, [limb, joints[6], -0.1], joints[6]+" decrease")
    #  }

    done = False
    while not done and not rospy.is_shutdown():
        c = input("Press [A] to enter input:")
        if c == 'A':
            joint_com = {}
            for joint in joints:
                c = float(input("Input delta rotation (radians) for {}:".format(joint)))
                if abs(c) > math.pi / 4:
                    c = 0
                    print("Rotation Too Large")
                joint_pos = set_j(limb, joint, c)
                joint_com[joint] = joint_pos
                print(c)

            print("Moving...")

            limb.set_joint_position_speed(0.3)
            for i in range(10):
                limb.set_joint_positions(joint_com)
                time.sleep(0.01)

            print("Done")
        else:
            exit()



def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    map_keyboard(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
