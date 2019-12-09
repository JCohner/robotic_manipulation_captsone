import modern_robotics as mr
import numpy as np
import pandas as pd

def to_csv(list,k, gtime, gstate, mode, end=False):
	traj = np.array(list).reshape(k,-1)
	df = pd.DataFrame(np.zeros((k,13)))
	df.iloc[:,:3] = traj[:,:3]
	df.iloc[:,3:6] = traj[:,4:7]
	df.iloc[:,6:9] = traj[:,8:11]
	df.iloc[:,9] = traj[:,3]
	df.iloc[:,10] = traj[:,7]
	df.iloc[:,11] = traj[:,11]
	df.iloc[gtime:, 12] = gstate
	if end:
		df.iloc[:gtime, 12] = 1

	df.to_csv("eggs.csv", header=False, index=False, mode = mode)
	return df.to_numpy()

def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k):
	method = 5 #5th order poly
	
	N = k
	
	traj = pd.DataFrame(np.zeros((6 * k,13)))
	traj_array = np.zeros((6 * k,4,4))

	#go to grip position
	Xstart = Tse_init #initial 
	Xend = np.matmul(Tsc_init, Tce_standoff) #standoff
	Tf = 10
	#this is the call to position our gripper over the piece 
	traj_s1 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method) 
	traj_array[:k,:,:] = traj_s1
	traj.iloc[:k,:] = to_csv(traj_s1, k, 0, 0, 'w')

	#standoff and grab
	Xstart = Xend #standoff
	Xend = np.matmul(Tsc_init, Tce_grasp) #grab
	Tf = 8
	#this is the call to position our gripper over the piece 
	traj_s2 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method) 
	traj_array[k:2*k,:,:] = traj_s2
	traj.iloc[k:2*k,:] = to_csv(traj_s2, k, 0, 0, 'a')

	#lift it back up
	Xstart = Xend #grab
	Xend = np.matmul(Tsc_init, Tce_standoff) #standoff
	Tf = 10
	#this is the call to position our gripper over the piece 
	traj_s3 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method) 
	traj_array[2*k:3*k,:,:] = traj_s3
	traj.iloc[2*k:3*k,:] = to_csv(traj_s3, k, 8, 1, 'a', end=True)

	#swing it over to final standoff position
	Xstart = Xend #standoff
	Xend = np.matmul(Tsc_final,Tce_standoff)
	Tf = 100
	traj_s4 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)
	traj_array[3*k:4*k,:,:] = traj_s4
	traj.iloc[3*k:4*k,:] = to_csv(traj_s4, k, 0, 1, 'a')

	#go to place
	Xstart = Xend #gripping standoff
	Xend = np.matmul(Tsc_final,Tce_grasp) #placement
	Tf = 20
	traj_s5 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)
	traj_array[4*k:5*k,:,:] = traj_s5
	traj.iloc[4*k:5*k,:] = to_csv(traj_s5, k, 0, 1, 'a')

	#let go, go back to standoff
	Xstart = Xend
	Xend = np.matmul(Tsc_final, Tce_standoff)
	Tf = 20
	traj_s6 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)
	traj_array[5*k:6*k,:,:] = traj_s6
	traj.iloc[5*k:6*k,:] = to_csv(traj_s6, k, 30, 0, 'a', end=True)

	traj.to_csv("eggs.csv", header=False, index=False)
	return traj_array, traj

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