#incldues from my own code
from TrajectoryGenerator import TrajectoryGenerator, to_csv
from NextState import NextState
from FeedbackControl import FeedbackControl, Jacobian_in_Body_Pinv

#standard python includes
import numpy as np
import pandas as pd
import modern_robotics as mr
import matplotlib.pyplot as plt
import sys
import logging
logger = logging.getLogger("my_log")
handler = logging.FileHandler('best.log')
formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)
logger.setLevel(logging.INFO)

def main():
	#initial configuration
	q0 = np.array([np.pi/6.,.2,-.2,
				   0,0,0.2,0,0,
				   0,0,0,0,
				   0])
	#Here are configurations to calcualte Tse from an initial configuration, the requested end implementation does not take advantage of this, but was useful in debugging
	B_list_arm = np.array([[0,0,0,0,0],
						  [0,-1,-1,-1,0],
						  [1,0,0,0,1],
						  [0,-0.5076,-0.3526,-0.2176,0],
						  [0.033,0,0,0,0],
						  [0,0,0,0,0]])

	Tsb = np.array([[np.cos(q0[0]),-np.sin(q0[0]),0,q0[1]],
					[np.sin(q0[0]),np.cos(q0[0]),0,q0[2]],
					[0,0,1,0.0963],
					[0,0,0,1]])

	Tb0 = np.array([[1,0,0,0.1662],
					[0,1,0,0],
					[0,0,1,0.0026],
					[0,0,0,1]])

	M0e = np.array([[1,0,0,0.033],
					[0,1,0,0],
					[0,0,1,0.5],
					[0,0,0,1]])
	T0e = mr.FKinBody(M0e, B_list_arm, q0[3:8])
	# Tse_init = np.matmul(np.matmul(Tsb,Tb0),T0e)

	#the requested initial end effector config for submission
	Tse_init = np.array([[0, 0, 1, 0.0],
					     [0, 1, 0, 0],
					     [-1, 0, 0, 0.653],
					     [0, 0, 0, 1,]])

	#initial and final positions of the cube along with standoff and grasping position of end effector relative to cube
	Tsc_init = np.array([[1, 0, 0, 1],
				   	 	 [0, 1, 0, 0],
				   	 	 [0, 0, 1, 0.025],
				   	 	 [0, 0, 0, 1,]])

	Tsc_final = np.array([[0, 1, 0, 0],
				   	 	  [-1, 0, 0, -1],
				   	 	  [0, 0, 1, 0.025],
				   	 	  [0, 0, 0, 1,]])

	Tce_grasp = np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), 0],
						  [0, 1, 0, 0],
						  [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0],
						  [0, 0, 0, 1]])

	Tce_standoff = np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), -.05],
						  [0, 1, 0, 0],
						  [-1/np.sqrt(2), 0, -1/np.sqrt(2), .05],
						  [0, 0, 0, 1]])

	#Definitons of constant and simulation parameterizing terms
	k = 200
	dt = 0.01

	#Terms for feeback control to calculate velocity
	omega_max = 5
	Kp = np.identity(4) * 30 #np.zeros((4,4))
	Ki = np.identity(4) * .1

	#generation of trajectory
	Te_traj, traj_df = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)

	#intiialize variables that will make outputting to csv easier
	config = pd.DataFrame(np.zeros((7*k,13)))
	xerr = np.zeros((6,7*k))
	config.iloc[0,:] = q0

	#simulation loop
	logger.info("Generating Trajectory...")
	for i in range(1,6*k-1):
		q = config.iloc[i-1,:].to_numpy()
		
		#get twist to next pose using feedback control
		Je_pinv, Tbe = Jacobian_in_Body_Pinv(q[:8])
		#get desired position
		Xd = Te_traj[i,:,:]
		#get next desired position
		Xd_next = Te_traj[i+1,:,:]
		#get current position 
		Tsb = np.array([[np.cos(q[0]),-np.sin(q[0]),0,q[1]],
					[np.sin(q[0]),np.cos(q[0]),0,q[2]],
					[0,0,1,0.0963],
					[0,0,0,1]])
		X = np.matmul(Tsb,Tbe)
		#get desired twist of end effector and corresponding calculated error
		Ve, Xerr = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt)
		xerr[:,i] = Xerr
		#calculate corresponding velocities
		vels = np.matmul(Je_pinv, Ve) 		
		#put vels in to get next state
		config.iloc[i,:] = NextState(q, vels, dt, omega_max) 
		if traj_df.iloc[i,12] == 1:
			config.iloc[i,12] = 1

	if (len(sys.argv) > 1):
		name = sys.argv[1]
	else:
		name = 'eggs'

	#output trajectory to csv
	logger.info('Outputting trajectory to csv')
	config.to_csv("{}.csv".format(name), header=False, index=False)

	#plot and output xerr
	xerr_df = pd.DataFrame(xerr)
	logger.info('Outputting error csv')
	xerr_df.to_csv("{}_error.csv".format(name), header=False, index=False)
	length = xerr.shape[1]
	
	tvec = np.arange(0,14,dt)
	for i in range(6):
		plt.plot(tvec,xerr[i,:])
	plt.legend(['w_x','w_y','w_z','v_x','v_y','v_z'])
	plt.title("error over time")
	plt.ylabel("error")
	plt.xlabel("time [s]") 
	plt.show()
if __name__ == '__main__':
	main()