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

alpha = 0.5

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

    N = 7 # number of control intervals
    dt = 1/30
    g = 9.81
    weights = [40, 40, 2, 2, 5, 5]

    angle_rate_limit = 0.1  
    angle_acc_limit = 0.02
    angle_limit = np.pi/10

    opti = Opti() # Optimization problem

    # ---- decision variables ---------
    X = opti.variable(6,N+1) # state trajectory
    xpos  = X[0,:]
    ypos  = X[1,:]
    xvel  = X[2,:]
    yvel  = X[3,:]
    pitch = X[4,:]
    roll  = X[5,:]
    U = opti.variable(2,N)   # control trajectory (throttle)
    pvel  = U[0,:]
    rvel  = U[1,:]

    f = lambda x,u: vertcat(x[0] +   dt*x[2],
                            x[1] +   dt*x[3],
                            x[2] - dt*g*x[4],
                            x[3] - dt*g*x[5],
                            x[4] +   dt*u[0],
                            x[5] +   dt*u[1])

    Q = DM.eye(6)
    for i in range(6):
        Q[i, i] = weights[i]
    P = Q
    R = DM.eye(2) * 0.5
    M = DM.eye(2) * 0.2

    cost = X[:, N].T@P@X[:, N]

    for k in range(N):
        x_next = f(X[:,k], U[:,k])
        opti.subject_to(X[:,k+1]==x_next)

        cost += X[:, k].T@Q@X[:, k] + U[:, k].T@R@U[:, k]

        if k < N-1:

            diff = U[:, k+1] - U[:, k]
            cost += diff.T@M@diff

            # opti.subject_to(opti.bounded(-angle_acc_limit,pvel[k+1] - pvel[k],angle_acc_limit))
            # opti.subject_to(opti.bounded(-angle_acc_limit,rvel[k+1] - rvel[k],angle_acc_limit))

    opti.subject_to(opti.bounded(-angle_limit,pitch,angle_limit)) # control is limited
    opti.subject_to(opti.bounded(-angle_limit,roll,angle_limit)) # control is limited

    opti.subject_to(opti.bounded(-angle_rate_limit,pvel,angle_rate_limit)) # control is limited
    opti.subject_to(opti.bounded(-angle_rate_limit,rvel,angle_rate_limit)) # control is limited

    opti.subject_to(xpos[0] ==x0[0])
    opti.subject_to(ypos[0] ==x0[1])
    opti.subject_to(xvel[0] ==x0[2])
    opti.subject_to(yvel[0] ==x0[3])
    opti.subject_to(pitch[0]==x0[4])
    opti.subject_to(roll[0] ==x0[5])

    opti.subject_to(opti.bounded(-angle_acc_limit,pvel[0] - u0[0],angle_acc_limit))
    opti.subject_to(opti.bounded(-angle_acc_limit,rvel[0] - u0[1],angle_acc_limit))


    opti.minimize(cost)

    p_opts = dict(print_time=False, verbose = False)
    s_opts = dict(print_level = 0)

    opti.solver("ipopt",p_opts,s_opts) # set numerical backend
    sol = opti.solve()   # actual solve

    return sol.value(pvel)[0], sol.value(rvel)[0]


def joint_vel(data, args):

    limb = args[0]
    desp = args[1]
    desr = args[2]

    joint_care = ['right_j3', 'right_j4']

    t = 0

    current_position = np.array([limb.joint_angles()[joint_name] for joint_name in joint_care])
    current_position[0] = -(current_position[0] - desp)
    current_position[1] = current_position[1] - desr

    current_velocity = np.array([limb.joint_velocities()[joint_name] for joint_name in joint_care])
    current_velocity[0] = -current_velocity[0]
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
    pvel, rvel = solve_cftoc([x, y, velocity_x, velocity_y, current_position[0], current_position[1]], prev_angle_vel)
    # timer.toc('Solve Time Took: ')

    prev_angle_vel[0] = pvel
    prev_angle_vel[1] = rvel

    dic_vel = {'right_j3':-pvel, 'right_j4':rvel}
    limb.set_joint_velocities(dic_vel)

    # print(dic_vel)

    print(pvel, rvel)

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
