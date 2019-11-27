from js import print_div
from js import document
from js import ImageData
from RobotRaconteur.Client import *
import numpy as np

from js import Plotly


def H_inv(H):							#inverse the homogeneous transformation matrix
	R=H[:2,:2]
	R=np.transpose(R)
	d=np.transpose(np.array([H[:2,2]]))
	d=-np.dot(R,d)
	H=np.concatenate((np.concatenate((R,d),axis=1),np.array([[0,0,1]])),axis=0)
	return H
def WebcamImageToMat(image):
    frame2=image.data.reshape([image.height, image.width, 3], order='C')
    return frame2

def new_frame(pipe_ep):
	global canvas, ctx
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		image=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		
		if (canvas == None):
			canvas = document.getElementById("image")
			ctx = canvas.getContext("2d")
		
		imageBytes=np.zeros(4*image.width*image.height, dtype=np.uint8)		#dtype essential here, IndexSizeError
		for y in range(image.height):
		
			for x in range(image.width):
			
				index1 = (x + image.width * y) * 4
				index2 = (x * 3 + image.step * y)
				imageBytes[index1] = image.data[index2 + 2]
				imageBytes[index1 + 1] = image.data[index2 + 1]
				imageBytes[index1 + 2] = image.data[index2]
				imageBytes[index1 + 3] = 255


		image_data=ImageData.new(bytes(imageBytes),image.width,image.height)
		ctx.putImageData(image_data, 0, 0,0,0,320,240)

async def client_plotly():

	uname=document.getElementById("uname").value
	psw=document.getElementById("psw").value

	credentials={"password":RR.RobotRaconteurVarValue(psw,"string")}

	try:
		inst=await RRN.AsyncConnectService('rr+ws://128.113.224.144:52222/?service=SmartCam',uname,credentials,None,None)
		Sawyer=await RRN.AsyncConnectService('rr+ws://128.113.224.144:8884/?service=Sawyer',uname,credentials,None,None)
		UR=await RRN.AsyncConnectService('rr+ws://128.113.224.144:2355/?service=Universal_Robot',uname,credentials,None,None)
		c_host=await RRN.AsyncConnectService('rr+ws://128.113.224.144:2366?service=Webcam',uname,credentials,None,None)
		c= await c_host.async_get_Webcams(0,None)

		p= await c.FrameStream.AsyncConnect(-1,None)
		global canvas, ctx
		canvas = document.getElementById("image")
		ctx = canvas.getContext("2d")
		print_div("Running!")
		canvas = document.getElementById("image")
		ctx = canvas.getContext("2d")

		while True:
			await plot(Sawyer,UR,inst)
			p.PacketReceivedEvent+=new_frame

			c.async_StartStreaming(None)


			await RRN.AsyncSleep(0.01,None)

	except:
		import traceback
		print_div(traceback.format_exc())
		raise

H_Sawyer=np.array([[0.99942127,0.03401641,0.01130381],[-0.03401641,0.99942127,0.05279206],[0,0,1]])
H_UR=np.array([[0.04223891, -0.99910754,  0.00630481],[0.99910754,  0.04223891, -1.3026918],[0,0,1]])

H_S_C=H_inv(H_Sawyer)
H_UR_C=H_inv(H_UR)


async def plot(Sawyer, UR, inst):

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
	pose_UR=ur_state[0].kin_chain_tcp


	# Draw x and y lists
	pose_Sawyer_C=np.dot(H_S_C,np.array([[pose_Sawyer[0]['position']['x']],[pose_Sawyer[0]['position']['y']],[1]]))
	pose_UR_C=np.dot(H_UR_C,np.array([[pose_UR[0]['position']['x']],[pose_UR[0]['position']['y']],[1]]))


	objects={ 'y': ys, 'x': xs ,'mode':'markers','name':'objects','type':'scatter','marker':{'size':10,'color':'#000000'}}
	UR5_robot={ 'y': pose_UR_C[1], 'x': pose_UR_C[0] ,'mode':'markers','name':'UR5_robot','type':'scatter','marker':{'size':10,'color':'#27e3d3'}}
	Sawyer_robot={ 'y': pose_Sawyer_C[1], 'x': pose_Sawyer_C[0] ,'mode':'markers','name':'Sawyer_robot','type':'scatter','marker':{'size':10,'color':'#e31010'}}

	Plotly.react('plot',[objects,UR5_robot,Sawyer_robot],)



	



RR.WebLoop.run(client_plotly())
