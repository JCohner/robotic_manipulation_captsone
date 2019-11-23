import modern_robotics as mr
import numpy as np
import pandas as pd

def to_csv(list,k, gtime, gstate, mode):
	traj = np.array(list).reshape(k,-1)
	df = pd.DataFrame(np.zeros((k,13)))
	df.iloc[:,:3] = traj[:,:3]
	df.iloc[:,3:6] = traj[:,4:7]
	df.iloc[:,6:9] = traj[:,8:11]
	df.iloc[:,9] = traj[:,3]
	df.iloc[:,10] = traj[:,7]
	df.iloc[:,11] = traj[:,11]
	df.iloc[gtime:, 12] = gstate

	df.to_csv("eggs.csv", header=False, index=False, mode = mode)

def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k):
	method = 5 #5th order poly
	Tf = 5
	N = k
	
	Xstart = Tse_init
	Xend = np.matmul(Tsc_init, Tce_standoff)
	#this is the call to position our gripper over the piece 
	traj_s1 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method) 
	to_csv(traj_s1, k, 0, 0, 'w')

	Xstart = Xend
	Xend = np.matmul(Tsc_init, Tce_grasp)
	Tf = 2
	#this is the call to position our gripper over the piece 
	traj_s2 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method) 
	to_csv(traj_s2, k, 50, 1, 'a')

	Xstart = Xend
	Xend = np.matmul(Tsc_final,Tce_grasp)
	Tf = 5
	traj_s3 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)
	to_csv(traj_s3, k, 0, 1, 'a')

	Xstart = Xend
	Xend = np.matmul(Tsc_final, Tce_standoff)
	Tf = 2
	traj_s4 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)
	to_csv(traj_s4, k, 0, 1, 'a')


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

	TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)