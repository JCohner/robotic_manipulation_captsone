from TrajectoryGenerator import TrajectoryGenerator, to_csv
from NextState import NextState
from FeedbackControl import FeedbackControl, Jacobian_in_Body_Pinv

import numpy as np
import pandas as pd
import modern_robotics as mr

import matplotlib.pyplot as plt
def main():
	q0 = np.array([np.pi/6.,.2,-.2,
				   0,0,0.2,0,0,
				   0,0,0,0,
				   0])

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

	Tse_init = np.array([[0, 0, 1, 0.0],
					     [0, 1, 0, 0],
					     [-1, 0, 0, 0.653],
					     [0, 0, 0, 1,]])
	# Tse_init = np.matmul(np.matmul(Tsb,Tb0),T0e)


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


	k = 200
	dt = 0.01
	omega_max = 5

	Kp = np.identity(4) * 3 #np.zeros((4,4))
	Ki = np.identity(4) * .1

	Te_traj, traj_df = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)

	print(traj_df.shape)

	config = pd.DataFrame(np.zeros((7*k,13)))
	xerr = np.zeros((6,7*k))
	config.iloc[0,:] = q0

	for i in range(1,6*k-1):
		q = config.iloc[i-1,:].to_numpy()
		
		#get twist to next pose using feedback control
		Je_pinv, Tbe = Jacobian_in_Body_Pinv(q[:8])
		Xd = Te_traj[i,:,:]
		Xd_next = Te_traj[i+1,:,:] 
		Tsb = np.array([[np.cos(q[0]),-np.sin(q[0]),0,q[1]],
					[np.sin(q[0]),np.cos(q[0]),0,q[2]],
					[0,0,1,0.0963],
					[0,0,0,1]])
		X = np.matmul(Tsb,Tbe)
		Ve, Xerr = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt)
		xerr[:,-i] = Xerr
		vels = np.matmul(Je_pinv, Ve) #TODO: make sure this is returning velocities in the same way NextState expects them
		#put vels in to get next state
		config.iloc[i,:] = NextState(q, vels, dt, omega_max) 
		if traj_df.iloc[i,12] == 1:
			config.iloc[i,12] = 1


	config.to_csv("chungus.csv", header=False, index=False)
if __name__ == '__main__':
	main()