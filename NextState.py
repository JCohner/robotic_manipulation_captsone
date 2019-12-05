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

#q - config 12 x 1, thetadot, thetadot - control vector 9 x 1, dt - timestep, omega_max - max angular velocity of arm joints and wheels 
def NextState(q, thetadot, dt, omega_max):
	q_new = 1

	#return the new configuration
	return q_new

def main():
	dt = 0.01
	t_total = 1
	tvec = np.arange(0,t_total,dt)
	print(tvec.shape)

if __name__ == '__main__':
	main()