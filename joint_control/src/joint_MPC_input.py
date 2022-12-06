import argparse

import rospy
import os
import numpy as np
import intera_interface
import intera_external_devices
from casadi import *

# import pyomo.environ as pyo
# from pyomo.opt import SolverStatus, TerminationCondition

from intera_interface import CHECK_VERSION
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
def solve_cftoc(x0, pvel_entered, rvel_entered):

    N = 5 # number of control intervals
    dt = 0.02
    g = 9.81
    angle_rate_limit = 0.2  
    angle_acc_limit = 0.4
    angle_limit = np.pi/2

    opti = Opti() # Optimization problem

    # ---- decision variables ---------
    X = opti.variable(2,N+1) # state trajectory
    xpos  = X[0,:]
    ypos  = X[1,:]
    U = opti.variable(2,N)   # control trajectory (throttle)
    pvel  = U[0,:]
    rvel  = U[1,:]

    f = lambda x,u: vertcat(x[0] +   dt*u[0],
                            x[1] +   dt*u[1])

    Q = P = DM.eye(2)
    R = DM.eye(2) * 0.02

    cost = X[:, N].T@P@X[:, N]

    for k in range(N):
        x_next = f(X[:,k], U[:,k])
        opti.subject_to(X[:,k+1]==x_next)

        cost += X[:, k].T@Q@X[:, k] + U[:, k].T@R@U[:, k]

        if k < N-1:
            opti.subject_to(opti.bounded(-angle_acc_limit*dt,pvel[k+1] - pvel[k],angle_acc_limit*dt))
            opti.subject_to(opti.bounded(-angle_acc_limit*dt,rvel[k+1] - rvel[k],angle_acc_limit*dt))

    opti.subject_to(opti.bounded(-angle_limit,xpos,angle_limit)) # control is limited
    opti.subject_to(opti.bounded(-angle_limit,ypos,angle_limit)) # control is limited

    opti.subject_to(opti.bounded(-angle_rate_limit,pvel,angle_rate_limit)) # control is limited
    opti.subject_to(opti.bounded(-angle_rate_limit,rvel,angle_rate_limit)) # control is limited

    opti.subject_to(xpos[0]==x0[0])
    opti.subject_to(ypos[0]==x0[1])
    opti.subject_to(pvel[0]==pvel_entered)
    opti.subject_to(rvel[0]==rvel_entered)


    opti.minimize(cost)

    # dt = T/N # length of a control interval
    # for k in range(N): # loop over control intervals
    #    # Runge-Kutta 4 integration
    #    k1 = f(X[:,k],         U[:,k])
    #    k2 = f(X[:,k]+dt/2*k1, U[:,k])
    #    k3 = f(X[:,k]+dt/2*k2, U[:,k])
    #    k4 = f(X[:,k]+dt*k3,   U[:,k])
    #    x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
    #    opti.subject_to(X[:,k+1]==x_next) # close the gaps

    # ---- path constraints -----------
    # limit = lambda pos: 1-sin(2*pi*pos)/2
    # opti.subject_to(speed<=limit(pos))   # track speed limit
    # opti.subject_to(opti.bounded(0,U,1)) # control is limited

    # ---- boundary conditions --------
    # opti.subject_to(pos[0]==0)   # start at position 0 ...
    # opti.subject_to(speed[0]==0) # ... from stand-still 
    # opti.subject_to(pos[-1]==1)  # finish line at position 1

    # # ---- misc. constraints  ----------
    # opti.subject_to(T>=0) # Time must be positive

    # ---- initial values for solver ---
    # opti.set_initial(speed, 1)
    # opti.set_initial(T, 1)

    # ---- solve NLP              ------
    p_opts = dict(print_time=False, verbose = False)
    s_opts = dict(print_level = 0)

    opti.solver("ipopt",p_opts,s_opts) # set numerical backend
    sol = opti.solve()   # actual solve

    # ---- post-processing        ------
    return sol.value(pvel)[1], sol.value(rvel)[1]


def joint_vel(side):

    limb = intera_interface.Limb(side)

    dic_vel = {name:np.zeros(len(limb.joint_names())) for name in limb.joint_names()}
    limb.set_joint_velocities(dic_vel)
    print(limb.joint_angles())

    t = 0

    r = rospy.Rate(50)

    joint_care = ['right_j3', 'right_j4']

    desp = limb.joint_angles()['right_j3']
    desr = limb.joint_angles()['right_j4']

    pvel_entered = 0
    rvel_entered = 0

    commp = 0
    commr = 0

    while not rospy.is_shutdown():

        if t == 100:
            commp = float(input('First Joint Angle: '))
            commr = float(input('Second Joint Angle:'))
            if abs(commp) > 0.5 or abs(commr) > 0.5:
                commp = 0
                commr = 0
            t = 0

        current_position = np.array([limb.joint_angles()[joint_name] for joint_name in joint_care])
        # current_velocity = np.array([limb.joint_velocities()[joint_name] for joint_name in limb.joint_names()])
        print(limb.joint_angles())
        current_position[0] -= desp + commp
        current_position[1] -= desr + commr


        pvel, rvel = solve_cftoc(current_position,pvel_entered, rvel_entered)

        pvel_entered = pvel
        rvel_entered = rvel

        dic_vel = {'right_j3':pvel, 'right_j4':rvel}
        limb.set_joint_velocities(dic_vel)


        t += 1
        r.sleep()



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
    joint_vel(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
    f.close()
