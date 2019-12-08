from TrajectoryGenerator import TrajectoryGenerator, to_csv
from NextState import NextState
from FeedbackControl import FeedbackControl, Jacobian_in_Body_Pinv

import numpy as np
import pandas as pd
import modern_robotics as mr

def main():
	Tse_init = np.array([[1, 0, 0, 0],
					     [0, 1, 0, 0],
					     [0, 0, 1, 0.4],
					     [0, 0, 0, 1,]])

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

	Tce_standoff = np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), -.025],
						  [0, 1, 0, 0],
						  [-1/np.sqrt(2), 0, -1/np.sqrt(2), .025],
						  [0, 0, 0, 1]])


	k = 100
	dt = 0.01

	Kp = np.zeros((4,4))
	Ki = np.zeros((4,4))

	Te_traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)

	q0 = np.array([0,0,0,
				   0,0,0.2,-1.6,
				   0,0,0,0,
				   0])

	config = pd.DataFrame(np.zeros((4*k,13)))
	config.iloc[0,:12] = q0

	for i in range(1,4*k-1):
		q = config.iloc[i-1,:].to_numpy()
		
		#get twist to next pose using feedback control
		Je_pinv, Tbe = Jacobian_in_Body_Pinv(q)
		Xd = Te_traj[i,:,:]
		Xd_next = Te_traj[i+1,:,:] 
		Tsb = np.array([[np.cos(q[0]),-np.sin(q[0]),0,q[1]],
					[np.sin(q[0]),np.cos(q[0]),0,q[2]],
					[0,0,1,0.0963],
					[0,0,0,1]])
		X = np.matmul(Tsb,Tbe)
		Ve = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt)

		vels = np.matmul(Je_pinv, Ve)
		#put vels in to get next state

if __name__ == '__main__':
	main()