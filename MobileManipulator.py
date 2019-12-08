from TrajectoryGenerator import TrajectoryGenerator, to_csv
from NextState import NextState
from FeedbackControl import FeedbackControl

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

	traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)

	q0 = np.array([0,0,0,
				   0,0,0.2,-1.6,
				   0,0,0,0,
				   0])

	config = pd.DataFrame(np.zeros((4*k,13)))
	config.iloc[0,:] = q0

	# for i in range(4*k-1):


if __name__ == '__main__':
	main()