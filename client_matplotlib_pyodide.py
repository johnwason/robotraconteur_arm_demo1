from js import print_div
from js import document
from RobotRaconteur.Client import *
from matplotlib import pyplot as plt
from general_robotics_toolbox import *
import numpy as np


ex=np.array([[1],[0],[0]])
ey=np.array([[0],[1],[0]])
ez=np.array([[0],[0],[1]])
H=np.concatenate((ez,-ey,-ey,-ey,-ez,-ey),axis=1)
p0=np.array([[0],[0],[0]])
p1=np.array([[0],[0],[0.089159]])
p2=np.array([[-0.425],[0],[0]])
p3=np.array([[-0.39225],[0],[0]])
p4=np.array([[0],[-0.10915],[0]])
p5=np.array([[0],[0],[-0.09465]])
p6=np.array([[0],[-0.0823],[0]])
P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)
joint_type=np.zeros(6)
UR_def=Robot(H,P,joint_type)


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

        await inst.async_update(None)        
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

H_Sawyer=np.array([[0.99942127,0.03401641,0.01130381],[-0.03401641,0.99942127,0.05279206],[0,0,1]])
H_UR=np.array([[0.04223891, -0.99910754,  0.00630481],[0.99910754,  0.04223891, -1.3026918],[0,0,1]])

H_S_C=H_inv(H_Sawyer)
H_UR_C=H_inv(H_UR)

q_Sawyer=[0,0,0,0,0,0,0]
pose_Sawyer=0

async def animate(i, Sawyer, UR, inst):
    global pose_Sawyer, q_UR, q_Sawyer
    # await inst.async_update(None)
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
    pose_UR=fwdkin(UR_def,q_UR).p
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
