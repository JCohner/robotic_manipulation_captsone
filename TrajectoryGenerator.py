import modern_robotics as mr
import numpy as np
import pandas as pd

def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k):
	method = 5 #5th order poly
	Tf = 5
	N = k
	
	Xstart = Tse_init
	Xend = np.matmul(Tsc_init, Tce_standoff)
	#this is the call to position our gripper over the piece 
	traj_s1 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method) 
	#shape it to a 2d array of (100, 16)
	traj_s1 = np.array(traj_s1).reshape(k,-1)
	
	df = pd.DataFrame(np.zeros((k,13)))
	#pack data frame to csv specifications 
	df.iloc[:,:3] = traj_s1[:,:3]
	df.iloc[:,3:6] = traj_s1[:,4:7]
	df.iloc[:,6:9] = traj_s1[:,8:11]
	df.iloc[:,9] = traj_s1[:,3]
	df.iloc[:,10] = traj_s1[:,7]
	df.iloc[:,11] = traj_s1[:,11]


	df.to_csv("eggs.csv", header=False, index=False)
if __name__ == '__main__':
	Tse_init = np.array([[1, 0, 0, 0],
					     [0, 1, 0, 0],
					     [0, 0, 1, 0.4],
					     [0, 0, 0, 1,]])

	Tsc_init = np.array([[1, 0, 0, 1],
				   	 	 [0, 1, 0, 0],
				   	 	 [0, 0, 1, 0.025],
				   	 	 [0, 0, 0, 1,]])

	Tsc_final = np.array([[0, 1, 0, 0],
				   	 	  [-1, 0, 0, 1],
				   	 	  [0, 0, 1, 0.025],
				   	 	  [0, 0, 0, 1,]])

	Tce_grasp = np.array([[0, 0, 1, -0.25],
						  [0, 1, 0, 0],
						  [-1, 0, 0, 0],
						  [0, 0, 0, 1]])

	Tce_standoff = np.array([[0, 0, 1, 0],
						  [0, 1, 0, 0],
						  [-1, 0, 0, 0],
						  [0, 0, 0, 1]])

	k = 100

	TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)