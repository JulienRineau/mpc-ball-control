#!/usr/bin/env python

import argparse

import rospy
import os
import numpy as np
import intera_interface
import intera_external_devices
from casadi import *
# from pytictoc import TicToc
from std_msgs.msg import String
from collections import deque


# timer = TicToc()

# import pyomo.environ as pyo
# from pyomo.opt import SolverStatus, TerminationCondition

from intera_interface import CHECK_VERSION

prev_point = np.zeros(2)
prev_angle_vel = np.zeros(2)

alpha = 0.75

'''
input_angles = np.zeros((2, 501))

with open('pitch_roll_inputs_vel.txt') as f:
    i = 0
    for line in f:
        tokens = line.split()
        input_angles[0, i] = tokens[0]
        input_angles[1, i] = tokens[1]
        i += 1
'''
def solve_cftoc(x0, u0):

    N = 10 # number of control intervals
    dt = 1/30
    g = 9.81
    weights = [30, 1, 10]

    angle_rate_limit = 0.2  
    angle_acc_limit = 0.01
    angle_limit = np.pi/6

    opti = Opti() # Optimization problem

    # ---- decision variables ---------
    X = opti.variable(3,N+1) # state trajectory
    ypos  = X[0,:]
    yvel  = X[1,:]
    roll  = X[2,:]
    U = opti.variable(1,N)   # control trajectory (throttle)
    rvel  = U[0,:]

    f = lambda x,u: vertcat(x[0] +   dt*x[1],
                            x[1] - dt*g*x[2],
                            x[2] +   dt*u[0])

    Q = DM.eye(3)
    for i in range(3):
        Q[i, i] = weights[i]
    P = Q
    R = DM.eye(1) * 0.02
    M = DM.eye(1) * 0.02

    cost = X[:, N].T@P@X[:, N]

    for k in range(N):
        x_next = f(X[:,k], U[:,k])
        opti.subject_to(X[:,k+1]==x_next)

        cost += X[:, k].T@Q@X[:, k] + U[:, k].T@R@U[:, k]

        if k < N-1:

            # diff = U[:, k+1] - U[:, k]
            # cost += diff.T@M@diff

            opti.subject_to(opti.bounded(-angle_acc_limit,rvel[k+1] - rvel[k],angle_acc_limit))

    opti.subject_to(opti.bounded(-angle_limit,roll,angle_limit)) # control is limited

    opti.subject_to(opti.bounded(-angle_rate_limit,rvel,angle_rate_limit)) # control is limited

    opti.subject_to(ypos[0] ==x0[0])
    opti.subject_to(yvel[0] ==x0[1])
    opti.subject_to(roll[0] ==x0[2])

    opti.subject_to(opti.bounded(-angle_acc_limit,rvel[0] - u0[1],angle_acc_limit))

    opti.minimize(cost)

    p_opts = dict(print_time=False, verbose = False)
    s_opts = dict(print_level = 0)

    opti.solver("ipopt",p_opts,s_opts) # set numerical backend
    sol = opti.solve()   # actual solve

    return sol.value(rvel)[0]


def joint_vel(data, args):

    limb = args[0]
    desp = args[1]
    desr = args[2]

    joint_care = ['right_j3', 'right_j4']

    t = 0

    current_position = np.array([limb.joint_angles()[joint_name] for joint_name in joint_care])
    current_position[0] = -(current_position[0] - desp)
    current_position[1] =  (current_position[1] - desr)

    current_velocity = np.array([limb.joint_velocities()[joint_name] for joint_name in joint_care])
    current_velocity[0] = -current_velocity[0]
    current_velocity[1] = -current_velocity[1]
    print('pitch velocity:', current_velocity[0]) 
    print('roll  velocity:', current_velocity[1]) 

    point = data.data.split()

    y = -float(point[0])/100
    x = -float(point[1])/100

    x = alpha*x + prev_point[0]*(1-alpha)
    y = alpha*y + prev_point[1]*(1-alpha)

    velocity_x = (x - prev_point[0])*30
    velocity_y = (y - prev_point[1])*30

    print('Current Position : {} {}'.format(x, y))
    print('Current Velocity : {} {}'.format(round(velocity_x, 3), round(velocity_y, 3)))

    # print(round(velocity_y, 3))

    prev_point[0] = x
    prev_point[1] = y

    # print(prev_point)

    # timer.tic()
    rvel = solve_cftoc([y, velocity_y, current_position[1]], prev_angle_vel)
    # timer.toc('Solve Time Took: ')
    prev_angle_vel[1] = rvel

    dic_vel = {'right_j3':0, 'right_j4':rvel}
    limb.set_joint_velocities(dic_vel)

    # print(dic_vel)

    print(rvel)

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

    limb = intera_interface.Limb(args.limb)

    desp = limb.joint_angles()['right_j3']
    desr = limb.joint_angles()['right_j4']

    # joint_vel(args.limb)
    rospy.Subscriber("string_pub", String, joint_vel, (limb, desp, desr))
    # joint_vel()
    print("Done.")
    rospy.spin()


if __name__ == '__main__':
    main()
    f.close()
