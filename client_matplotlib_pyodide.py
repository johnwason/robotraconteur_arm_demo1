from js import print_div
from js import document
from RobotRaconteur.Client import *
from matplotlib import pyplot as plt
from math import *
import cmath
import numpy as np


def H_inv(H):							#inverse the homogeneous transformation matrix
    R=H[:2,:2]
    R=np.transpose(R)
    d=np.transpose(np.array([H[:2,2]]))
    d=-np.dot(R,d)
    H=np.concatenate((np.concatenate((R,d),axis=1),np.array([[0,0,1]])),axis=0)
    return H

async def client_matplotlib():

    uname=document.getElementById("uname").value
    psw=document.getElementById("psw").value

    credentials={"password":RR.RobotRaconteurVarValue(psw,"string")}

    try:
        inst=await RRN.AsyncConnectService('rr+ws://128.113.224.57:52222/?service=SmartCam',uname,credentials,None,None)
        Sawyer=await RRN.AsyncConnectService('rr+ws://128.113.224.57:8884/?service=Sawyer',uname,credentials,None,None)
        UR=await RRN.AsyncConnectService('rr+ws://128.113.224.57:2355/?service=Universal_Robot',uname,credentials,None,None)

        # await inst.async_update(None)        
        print_div("Running!")

        global fig
        global ax
        fig = plt.figure()
        fig.show()
        ax = fig.add_subplot(1, 1, 1)
        while True:
            await animate(0,Sawyer,UR,inst)
            await RRN.AsyncSleep(0.01,None)

    except:
        import traceback
        print_div(traceback.format_exc())
        raise


d1 = 0.089159 
d2 = d3 = 0
d4 = 0.10915
d5 = 0.09465
d6 = 0.0823

a1 = a4 = a5 = a6 = 0
a2 = -0.425
a3 = -0.39225
d = np.array([d1, d2, d3, d4, d5, d6]) # unit: mm
a = np.array([a1, a2, a3, a4, a5, a6]) # unit: mm
alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]) # unit: radian


def select(q_sols, q_d, w=[1]*6):
    """Select the optimal solutions among a set of feasible joint value 
       solutions.
    Args:
        q_sols: A set of feasible joint value solutions (unit: radian)
        q_d: A list of desired joint value solution (unit: radian)
        w: A list of weight corresponding to robot joints
    Returns:
        A list of optimal joint value solution.
    """

    error = []
    for q in q_sols:
        error.append(sum([w[i] * (q[i] - q_d[i]) ** 2 for i in range(6)]))
    
    return q_sols[error.index(min(error))]


def R2q(R):
    """
    Converts a 3 x 3 rotation matrix into a quaternion.  Quaternion is
    returned in the form q = [q0;qv].
    
    :type    R: numpy.array
    :param   R: 3 x 3 rotation matrix
    :rtype:  numpy.array
    :return: the quaternion as a 4 x 1 vector q = [q0;qv] 
      
    """
    
    tr = np.trace(R)
    if tr > 0:
        S = 2*sqrt(tr + 1)
        q = np.array([(0.25*S), \
                      ((R[2,1] - R[1,2]) / S), \
                      ((R[0,2] - R[2,0]) / S), \
                      ((R[1,0] - R[0,1]) / S)])
                      
    elif (R[0,0] > R[1,1] and R[0,0] > R[2,2]):
        S = 2*sqrt(1 + R[0,0] - R[1,1] - R[2,2])
        q = np.array([((R[2,1] - R[1,2]) / S), \
                      (0.25*S), \
                      ((R[0,1] + R[1,0]) / S), \
                      ((R[0,2] + R[2,0]) / S)])
    elif (R[1,1] > R[2,2]):
        S = 2*sqrt(1 - R[0,0] + R[1,1] - R[2,2])
        q = np.array([((R[0,2] - R[2,0]) / S), \
                      ((R[0,1] + R[1,0]) / S), \
                      (0.25*S), \
                      ((R[1,2] + R[2,1]) / S)])
    else:
        S = 2*sqrt(1 - R[0,0] - R[1,1] + R[2,2])
        q = np.array([((R[1,0] - R[0,1]) / S), \
                      ((R[0,2] + R[2,0]) / S), \
                      ((R[1,2] + R[2,1]) / S), \
                      (0.25*S)])
    return q

def HTM(i, theta):
    """Calculate the HTM between two links.
    Args:
        i: A target index of joint value. 
        theta: A list of joint value solution. (unit: radian)
    Returns:
        An HTM of Link l w.r.t. Link l-1, where l = i + 1.
    """

    Rot_z = np.matrix(np.identity(4))
    Rot_z[0, 0] = Rot_z[1, 1] = np.cos(theta[i])
    Rot_z[0, 1] = -np.sin(theta[i])
    Rot_z[1, 0] = np.sin(theta[i])

    Trans_z = np.matrix(np.identity(4))
    Trans_z[2, 3] = d[i]

    Trans_x = np.matrix(np.identity(4))
    Trans_x[0, 3] = a[i]

    Rot_x = np.matrix(np.identity(4))
    Rot_x[1, 1] = Rot_x[2, 2] = np.cos(alpha[i])
    Rot_x[1, 2] = -np.sin(alpha[i])
    Rot_x[2, 1] = np.sin(alpha[i])

    A_i = Rot_z * Trans_z * Trans_x * Rot_x
        
    return A_i


def fwd_kin(theta):

    T_06 = np.matrix(np.identity(4))

    for i in range(6):
        T_06 *= HTM(i, theta)
    orientation=R2q(T_06[:3,:3])
    position=T_06[:3,3]
    return T_06

def inv_kin(T_06, q_d):
    """Solve the joint values based on an HTM.
    Args:
        p: A pose.
        q_d: A list of desired joint value solution 
             (unit: radian).
        i_unit: Output format. 'r' for radian; 'd' for degree.
        o_unit: Output format. 'r' for radian; 'd' for degree.
    Returns:
        A list of optimal joint value solution.
    """

    


    # Initialization of a set of feasible solutions
    theta = np.zeros((8, 6))
 
    # theta1
    P_05 = T_06[0:3, 3] - d6 * T_06[0:3, 2]
    phi1 = atan2(P_05[1], P_05[0])
    phi2 = acos(d4 / sqrt(P_05[0] ** 2 + P_05[1] ** 2))
    theta1 = [pi / 2 + phi1 + phi2, pi / 2 + phi1 - phi2]
    theta[0:4, 0] = theta1[0]
    theta[4:8, 0] = theta1[1]
  
    # theta5
    P_06 = T_06[0:3, 3]
    theta5 = []
    for i in range(2):
        theta5.append(acos((P_06[0] * sin(theta1[i]) - P_06[1] * cos(theta1[i]) - d4) / d6))
    for i in range(2):
        theta[2*i, 4] = theta5[0]
        theta[2*i+1, 4] = -theta5[0]
        theta[2*i+4, 4] = theta5[1]
        theta[2*i+5, 4] = -theta5[1]
  
    # theta6
    T_60 = np.linalg.inv(T_06)
    theta6 = []
    for i in range(2):
        for j in range(2):
            s1 = sin(theta1[i])
            c1 = cos(theta1[i])
            s5 = sin(theta5[j])
            theta6.append(atan2((-T_60[1, 0] * s1 + T_60[1, 1] * c1) / s5, (T_60[0, 0] * s1 - T_60[0, 1] * c1) / s5))
    for i in range(2):
        theta[i, 5] = theta6[0]
        theta[i+2, 5] = theta6[1]
        theta[i+4, 5] = theta6[2]
        theta[i+6, 5] = theta6[3]

    # theta3, theta2, theta4
    for i in range(8):  
        # theta3
        T_46 = HTM(4, theta[i]) * HTM(5, theta[i])
        T_14 = np.linalg.inv(HTM(0, theta[i])) * T_06 * np.linalg.inv(T_46)
        P_13 = T_14 * np.array([[0, -d4, 0, 1]]).T - np.array([[0, 0, 0, 1]]).T
        if i in [0, 2, 4, 6]:
            theta[i, 2] = -cmath.acos((np.linalg.norm(P_13) ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)).real
            theta[i+1, 2] = -theta[i, 2]
        # theta2
        theta[i, 1] = -atan2(P_13[1], -P_13[0]) + asin(a3 * sin(theta[i, 2]) / np.linalg.norm(P_13))
        # theta4
        T_13 = HTM(1, theta[i]) * HTM(2, theta[i])
        T_34 = np.linalg.inv(T_13) * T_14
        theta[i, 3] = atan2(T_34[1, 0], T_34[0, 0])       

    theta = theta.tolist()

    # Select the most close solution
    q_sol = select(theta, q_d)

    return q_sol

H_Sawyer=np.array([[0.99942127,0.03401641,0.01130381],[-0.03401641,0.99942127,0.05279206],[0,0,1]])
H_UR=np.array([[0.04223891, -0.99910754,  0.00630481],[0.99910754,  0.04223891, -1.3026918],[0,0,1]])

H_S_C=H_inv(H_Sawyer)
H_UR_C=H_inv(H_UR)

q_Sawyer=[0,0,0,0,0,0,0]
pose_Sawyer=0

async def animate(i, Sawyer, UR, inst):
    global pose_Sawyer, q_UR, q_Sawyer
    await inst.async_update(None)
    xs = []
    ys = []

    # Add x and y to lists
    for obj in await inst.async_get_objects(None):
        xs.append(obj.x/1000.)
        ys.append(obj.y/1000.)

    sawyer_state = await Sawyer.robot_state.AsyncPeekInValue(None)
    ur_state = await UR.robot_state.AsyncPeekInValue(None)

    pose_Sawyer=sawyer_state[0].kin_chain_tcp
    q_Sawyer=sawyer_state[0].joint_position

    q_UR=ur_state[0].joint_position

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys,'ro')
    pose_Sawyer_C=np.dot(H_S_C,np.array([[pose_Sawyer[0]['position']['x']],[pose_Sawyer[0]['position']['y']],[1]]))
    ax.plot(pose_Sawyer_C[0],pose_Sawyer_C[1],'ro',color='blue')
    pose_UR=fwd_kin(q_UR)
    pose_UR_C=np.dot(H_UR_C,np.array([[pose_UR[0,3]],[pose_UR[1,3]],[1]]))
    ax.plot(pose_UR_C[0],pose_UR_C[1],'ro',color='blue')
    ax.set(xlim=(0, 1.7), ylim=(-1, 2))
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)

    text_UR = '\n'.join((
    r'$q1=%.2f$' % (q_UR[0], ),
    r'$q2=%.2f$' % (q_UR[1], ),
    r'$q3=%.2f$' % (q_UR[2], ),
    r'$q4=%.2f$' % (q_UR[3], ),
    r'$q5=%.2f$' % (q_UR[4], ),
    r'$q6=%.2f$' % (q_UR[5], )))

    ax.text(0.75, 0.95, text_UR, transform=ax.transAxes, fontsize=14,
            verticalalignment='top', bbox=props)
    text_Sawyer = '\n'.join((
    r'$q1=%.2f$' % (q_Sawyer[0], ),
    r'$q2=%.2f$' % (q_Sawyer[1], ),
    r'$q3=%.2f$' % (q_Sawyer[2], ),
    r'$q4=%.2f$' % (q_Sawyer[3], ),
    r'$q5=%.2f$' % (q_Sawyer[4], ),
    r'$q6=%.2f$' % (q_Sawyer[5], ),
    r'$q7=%.2f$' % (q_Sawyer[6], )))


    # place a text box in upper left in axes coords
    ax.text(0.02, 0.5, text_Sawyer, transform=ax.transAxes, fontsize=14,
            bbox=props)

    text_Sawyer_end = '\n'.join((
    r'$x=%.2f$' % (pose_Sawyer_C[0], ),
    r'$y=%.2f$' % (pose_Sawyer_C[1], )))
    ax.text(0.28, 0.7, text_Sawyer_end, transform=ax.transAxes, fontsize=14,
            bbox=props)
    text_UR_end = '\n'.join((
    r'$x=%.2f$' % (pose_UR_C[0], ),
    r'$y=%.2f$' % (pose_UR_C[1], )))
    ax.text(0.515, 0.7, text_UR_end, transform=ax.transAxes, fontsize=14,
            bbox=props)


RR.WebLoop.run(client_matplotlib())
