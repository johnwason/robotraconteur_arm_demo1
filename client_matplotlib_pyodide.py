from js import print_div
from js import document
from RobotRaconteur.Client import *
from matplotlib import pyplot as plt
import numpy as np
import math
import warnings

###########################RPI general toolbox

def hat(k):
	"""
	Returns a 3 x 3 cross product matrix for a 3 x 1 vector
	
			 [  0 -k3  k2]
	 khat =  [ k3   0 -k1]
			 [-k2  k1   0]
	
	:type    k: numpy.array
	:param   k: 3 x 1 vector
	:rtype:  numpy.array
	:return: the 3 x 3 cross product matrix    
	"""
	
	khat=np.zeros((3,3))
	khat[0,1]=-k[2]
	khat[0,2]=k[1]
	khat[1,0]=k[2]
	khat[1,2]=-k[0]
	khat[2,0]=-k[1]
	khat[2,1]=k[0]    
	return khat


def rot(k, theta):
	"""
	Generates a 3 x 3 rotation matrix from a unit 3 x 1 unit vector axis
	and an angle in radians using the Euler-Rodrigues formula
	
		R = I + sin(theta)*hat(k) + (1 - cos(theta))*hat(k)^2
		
	:type    k: numpy.array
	:param   k: 3 x 1 unit vector axis
	:type    theta: number
	:param   theta: rotation about k in radians
	:rtype:  numpy.array
	:return: the 3 x 3 rotation matrix 
		
	"""
	I = np.identity(3)
	khat = hat(k)
	khat2 = khat.dot(khat)
	return I + math.sin(theta)*khat + (1.0 - math.cos(theta))*khat2



class Robot(object):
	"""
	Holds the kinematic information for a single chain robot
	
	:attribute H: A 3 x N matrix containing the direction the joints as unit vectors, one joint per column
	:attribute P: A 3 x (N + 1) matrix containing the distance vector from i to i+1, one vector per column
	:attribute joint_type: A list of N numbers containing the joint type. 0 for rotary, 1 for prismatic, 2 and 3 for mobile
	:attribute joint_lower_limit: A list of N numbers containing the joint lower limits. Optional
	:attribute joint_upper_limit: A list of N numbers containing the joint upper limits. Optional
	:attribute joint_vel_limit: A list of N numbers containing the joint velocity limits. Optional
	:attribute joint_acc_limit: A list of N numbers containing the joint acceleration limits. Optional
	:attribute M: A list of N, 6 x 6 spatial inertia matrices for the links. Optional
	:attribute R_tool: A 3 x 3 rotation matrix for the tool frame. Optional
	:attribute p_tool: A 3 x 1 vector for the tool frame. Optional
	:attribute joint_names: A list of N strings containing the names of the joints if loaded from URDF. Optional
	:attribute root_link_name: A string containing the name of the kinematic chain root link if loaded from URDF. Optional
	:attribute tip_link_name: A string containing the name of the kinematic chain tip link if loaded from URDF. Optional
	
	"""
	
	
	def __init__(self, H, P, joint_type, joint_lower_limit = None, joint_upper_limit = None, joint_vel_limit = None, joint_acc_limit = None, M = None, \
				 R_tool=None, p_tool=None, joint_names = None, root_link_name = None, tip_link_name = None):
		
		"""
		Construct a Robot object holding the kinematic information for a single chain robot
	
		:type  H: numpy.array
		:param H: A 3 x N matrix containing the direction the joints as unit vectors, one joint per column
		:type  H: numpy. array
		:param P: A 3 x (N + 1) matrix containing the distance vector from i to i+1, one vector per column
		:type  joint_type: list or numpy.array
		:param joint_type: A list or array of N numbers containing the joint type. 0 for rotary, 1 for prismatic, 2 and 3 for mobile
		:type  joint_lower_limit: list or numpy.array
		:param joint_lower_limit: A list or array of N numbers containing the joint type minimums. Optional
		:type  joint_upper_limit: list or numpy.array
		:param joint_upper_limit: A list or array of N numbers containing the joint type maximums. Optional
		:type  joint_vel_limit: list or numpy.array
		:param joint_vel_limit: A list of N numbers containing the joint velocity limits. Optional
		:type  joint_acc_limit: list or numpy.array
		:param joint_acc_limit: A list of N numbers containing the joint acceleration limits. Optional
		:type  M: list of numpy.array
		:param M: A list of N, 6 x 6 spatial inertia matrices for the links. Optional
		:type  R_tool: numpy.array
		:param R_tool: A 3 x 3 rotation matrix for the tool frame. Optional
		:type  p_tool: numpy.array
		:param p_tool: A 3 x 1 vector for the tool frame. Optional
		:type  joint_names: list of string
		:param joint_names: A list of N strings containing the names of the joints if loaded from URDF. Optional
		:type  root_link_name: string
		:param root_link_name: A string containing the name of the kinematic chain root link if loaded from URDF. Optional
		:type  tip_link_name: string
		:param tip_link_name: A string containing the name of the kinematic chain tip link if loaded from URDF. Optional
	
		"""
		
		
		for i in range(H.shape[1]):
			assert (np.isclose(np.linalg.norm(H[:,i]), 1))        
		
		for j in joint_type:            
			assert (j in [0,1,2,3])                
		
		assert (H.shape[0] == 3 and P.shape[0] == 3)
		assert (H.shape[1] + 1 == P.shape[1] and H.shape[1] == len(joint_type))
		
		if (joint_lower_limit is not None and joint_upper_limit is not None):
			assert (len(joint_lower_limit) == len(joint_type))
			assert (len(joint_upper_limit) == len(joint_type))
			self.joint_lower_limit=joint_lower_limit
			self.joint_upper_limit=joint_upper_limit
		else:
			self.joint_lower_limit=None
			self.joint_upper_limit=None
			
		if (joint_vel_limit is not None):
			assert (len(joint_vel_limit) == len(joint_type))
			self.joint_vel_limit=joint_vel_limit
		else:
			self.joint_vel_limits=None
			
		if (joint_acc_limit is not None):
			assert (len(joint_acc_limit) == len(joint_type))
			self.joint_acc_limit=joint_acc_limit
		else:
			self.joint_acc_limits=None
			   
		if M is not None:
			assert (len(M) == H.shape[1])
			for m in M:
				assert (m.shape == (6,6))
			self.M = M
		else:
			self.M=None
		
		if R_tool is not None and p_tool is not None:
			self.R_tool = R_tool
			self.p_tool = p_tool
		else:
			self.R_tool = None
			self.p_tool = None        
		
		self.H = H
		self.P = P
		self.joint_type = joint_type
		
		if joint_names is not None:
			assert len(joint_names) == len(joint_type)
		self.joint_names = joint_names
		self.root_link_name = root_link_name
		self.tip_link_name = tip_link_name
		
	
			
class Transform(object):
	"""
	Holds a transform consisting of a rotation matrix and a vector
	
	:attribute R: The 3 x 3 rotation matrix
	:attribute p: The 3 x 1 position vector
	
	Note: Transform objects are also used to represent the pose of links 
	"""
	
	
	def __init__(self, R, p, parent_frame_id=None, child_frame_id=None):
		"""
		Construct a Transform object consisting of a rotation matrix and a vector
	
		:type  R: numpy.array
		:param R: The 3 x 3 rotation matrix
		:type  p: numpy.array
		:param p: The 3 x 1 position vector
		"""    
				
		assert (np.shape(R) == (3,3))
		assert (np.shape(p) == (3,) or np.shape(p) ==(3,1))
		
		self.R=np.array(R)
		self.p=np.reshape(p,(3,))
		self.parent_frame_id=parent_frame_id
		self.child_frame_id=child_frame_id
		
	def __mul__(self, other):
		R = np.dot(self.R, other.R)
		p = self.p + np.dot(self.R, other.p)
		return Transform(R,p,self.parent_frame_id, other.child_frame_id)
	
	def __eq__(self, other):
		#Use "np.isclose" because of numerical accuracy issues
		return np.all(np.isclose(self.R, other.R, 1e-6)) \
			and np.all(np.isclose(self.p, other.p, 1e-6))
			
	def __neq__(self, other):
		return not self.__eq__(other)
	
	def inv(self):
		R=np.transpose(self.R)
		p=-np.dot(R,self.p)
		return Transform(R,p,self.child_frame_id, self.parent_frame_id)
	
	def __repr__(self):
		r = ["Transform(", \
			"    R = " + np.array_repr(self.R, precision=4, suppress_small=True).replace('\n', '\n' + ' '*8), \
			"    p = " + np.array_repr(self.p, precision=4, suppress_small=True)]
		if self.parent_frame_id is not None:
			r.append("    parent_frame_id = \"" + self.parent_frame_id + "\"")
		if self.child_frame_id is not None:
			r.append("    child_frame_id = \"" + self.child_frame_id + "\"")
		r.append(")\n")        
		return "\n".join(r)
	
	def __str__(self):
		r = ["R = " + np.array_str(self.R, precision=4, suppress_small=True).replace('\n', '\n' + ' '*4), \
		  "p = " + np.array_str(self.p, precision=4, suppress_small=True)]
		if self.parent_frame_id is not None:
			r.append("parent_frame_id = \"" + self.parent_frame_id + "\"")
		if self.child_frame_id is not None:
			r.append("child_frame_id = \"" + self.child_frame_id + "\"")
		r.append("\n")        
		return "\n".join(r)  
	
def fwdkin(robot, theta):
	"""
	Computes the pose of the robot tool flange based on a Robot object
	and the joint angles.
	
	:type    robot: Robot
	:param   robot: The robot object containing kinematic information
	:type    theta: numpy.array
	:param   theta: N x 1 array of joint angles. Must have same number of joints as Robot object
	:rtype:  Transform
	:return: The Pose of the robot tool flange    
	"""    
	
	if (robot.joint_lower_limit is not None and robot.joint_upper_limit is not None):
		assert np.greater_equal(theta, robot.joint_lower_limit).all(), "Specified joints out of range"
		assert np.less_equal(theta, robot.joint_upper_limit).all(), "Specified joints out of range"
	
	p = robot.P[:,[0]]
	R = np.identity(3)
	for i in range(len(robot.joint_type)):
		if (robot.joint_type[i] == 0 or robot.joint_type[i] == 2):
			R = R.dot(rot(robot.H[:,[i]],theta[i]))
		elif (robot.joint_type[i] == 1 or robot.joint_type[i] == 3):
			p = p + theta[i] * R.dot(robot.H[:,[i]])
		p = p + R.dot(robot.P[:,[i+1]])
		
	p=np.reshape(p,(3,))
		
	if robot.R_tool is not None and robot.p_tool is not None:
		p = p + R.dot(robot.p_tool)
		R = R.dot(robot.R_tool)  
	
	return Transform(R, p)


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


async def animate(i, Sawyer, UR, inst):

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
	pose_UR_C=np.dot(H_UR_C,np.array([[pose_UR[0]],[pose_UR[1]],[1]]))
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
